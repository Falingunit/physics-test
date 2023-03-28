using Microsoft.Xna.Framework;

namespace PhysicsEngine
{
    public sealed class World
    {
        public static readonly float MinBodySize = 0.01f * 0.01f;
        public static readonly float MaxBodySize = 64f * 64f;

        public static readonly float MinDensity = 0.5f;
        public static readonly float MaxDensity = 21.4f;

        public static readonly int MinIterations = 1;
        public static readonly int MaxIterations = 128;

        public static readonly float VerySmallAmount = 0.0005f;

        private List<RigidBody> bodyList;
        private List<Constraint> constraintList;
        private Vector2 gravity;
        private List<(int, int)> contactPairs;

        private Vector2[] contactList;
        private Vector2[] impulseList;
        private Vector2[] frictionImpulseList;
        private Vector2[] raList;
        private Vector2[] rbList;
        private float[] jList;

        public int BodyCount
        {
            get { return bodyList.Count; }
        }

        public World()
        {
            gravity = new Vector2(0f, -9.81f);
            bodyList = new List<RigidBody>();
            constraintList = new List<Constraint>();
            contactPairs = new List<(int, int)>();

            this.contactList = new Vector2[2];
            this.impulseList = new Vector2[2];
            this.frictionImpulseList = new Vector2[2];
            this.raList = new Vector2[2];
            this.rbList = new Vector2[2];
            this.jList = new float[2];
        }

        public void AddBody(RigidBody body)
        {
            bodyList.Add(body);
        }

        public bool RemoveBody(RigidBody body)
        {
            return bodyList.Remove(body);
        }
        public Constraint CreateStaticConstraint(Vector2 constraintCenter, RigidBody body, float constraintMax, float constraintMin, float maxVelocity)
        {
            Constraint constraint = new Constraint(null, body, constraintCenter, constraintMax, constraintMin, maxVelocity, true);
            this.constraintList.Add(constraint);
            return constraint;
        }

        public Constraint CreateDynamicConstraint(RigidBody bodyA, RigidBody bodyB, float constraintMax, float constraintMin, float maxVelocity)
        {
            Constraint constraint = new Constraint(bodyA, bodyB, Vector2.Zero, constraintMax, constraintMin, maxVelocity, false);
            this.constraintList.Add(constraint);
            return constraint;
        }

        public bool GetBody(int index, out RigidBody? body)
        {
            body = null;

            if (index < 0 || index >= bodyList.Count)
            {
                return false;
            }

            body = bodyList[index];
            return true;
        }

        public void Step(float dt, int totalIterations)
        {
            totalIterations = Math.Clamp(totalIterations, MinIterations, MaxIterations);

            for (int currentIteration = 0; currentIteration < totalIterations; currentIteration++)
            {
                contactPairs.Clear();
                StepBodies(dt, totalIterations);
                this.BroadPhase();
                this.NarrowPhase();
                for (int i = 0; i < constraintList.Count; i++)
                {
                    constraintList[i].Update();
                }
            }
        }

        private void BroadPhase()
        {
            for (int i = 0; i < bodyList.Count - 1; i++)
            {
                RigidBody bodyA = bodyList[i];
                AABB aabbA = bodyA.GetAABB();

                for (int j = i + 1; j < bodyList.Count; j++)
                {
                    RigidBody bodyB = bodyList[j];
                    AABB aabbB = bodyB.GetAABB();

                    if (bodyA.IsStatic && bodyB.IsStatic)
                    {
                        continue;
                    }

                    if (!Collisions.IntersectAABBs(aabbA, aabbB))
                    {
                        continue;
                    }

                    this.contactPairs.Add((i, j));

                }
            }
        }

        private void NarrowPhase()
        {
            for (int i = 0; i < contactPairs.Count; i++)
            {
                (int, int) pair = this.contactPairs[i];
                RigidBody bodyA = this.bodyList[pair.Item1];
                RigidBody bodyB = this.bodyList[pair.Item2];

                if (Collisions.Collide(bodyA, bodyB, out Vector2 normal, out float depth))
                {
                    this.SeperateBodies(bodyA, bodyB, normal * depth);
                    Collisions.FindContactPoints(bodyA, bodyB, out Vector2 contact1, out Vector2 contact2, out int contactCount);
                    Manifold manifold = new Manifold(bodyA, bodyB, normal, depth, contact1, contact2, contactCount);
                    this.ResolveCollisionWithRotationAndFricton(in manifold);
                }

            }
        }

        private void StepBodies(float dt, int totalIterations)
        {
            for (int i = 0; i < bodyList.Count; i++)
            {
                bodyList[i].Step(dt, gravity, totalIterations);
            }
        }

        private void SeperateBodies(RigidBody bodyA, RigidBody bodyB, Vector2 mtv)
        {
            if (bodyA.IsStatic)
            {
                bodyB.Move(mtv);
            }
            else if (bodyB.IsStatic)
            {
                bodyA.Move(-mtv);
            }
            else
            {
                bodyA.Move(-mtv / 2f);
                bodyB.Move(mtv / 2f);
            }
        }

        public void ResolveCollisionBasic(in Manifold manifold)
        {
            RigidBody bodyA = manifold.BodyA;
            RigidBody bodyB = manifold.BodyB;
            Vector2 normal = manifold.Normal;
            float depth = manifold.Depth;

            Vector2 relativeVelocity = bodyB.LinearVelocity - bodyA.LinearVelocity;

            if (Vector2.Dot(relativeVelocity, normal) > 0f)
            {
                return;
            }

            float e = MathF.Min(bodyA.Restituition, bodyB.Restituition);

            float j = -(1f + e) * Vector2.Dot(relativeVelocity, normal);
            j /= bodyA.InvMass + bodyB.InvMass;

            Vector2 impulse = j * normal;

            bodyA.LinearVelocity -= impulse * bodyA.InvMass;
            bodyB.LinearVelocity += impulse * bodyB.InvMass;
        }

        private void ResolveCollisionWithRotation(in Manifold manifold)
        {
            RigidBody bodyA = manifold.BodyA;
            RigidBody bodyB = manifold.BodyB;
            Vector2 normal = manifold.Normal;
            Vector2 contact1 = manifold.Contact1;
            Vector2 contact2 = manifold.Contact2;
            int contactCount = manifold.ContactCount;

            float e = MathF.Min(bodyA.Restituition, bodyB.Restituition);

            this.contactList[0] = contact1;
            this.contactList[1] = contact2;

            for (int i = 0; i < contactCount; i++)
            {
                this.impulseList[i] = Vector2.Zero;
                this.raList[i] = Vector2.Zero;
                this.rbList[i] = Vector2.Zero;
            }

            for (int i = 0; i < contactCount; i++)
            {
                Vector2 ra = contactList[i] - bodyA.Position;
                Vector2 rb = contactList[i] - bodyB.Position;

                raList[i] = ra;
                rbList[i] = rb;

                Vector2 raPerp = new Vector2(-ra.Y, ra.X);
                Vector2 rbPerp = new Vector2(-rb.Y, rb.X);

                Vector2 angularLinearVelocityA = raPerp * bodyA.AngularVelocity;
                Vector2 angularLinearVelocityB = rbPerp * bodyB.AngularVelocity;

                Vector2 relativeVelocity =
                    (bodyB.LinearVelocity + angularLinearVelocityB) -
                    (bodyA.LinearVelocity + angularLinearVelocityA);

                float contactVelocityMag = Vector2.Dot(relativeVelocity, normal);

                if (contactVelocityMag > 0f)
                {
                    continue;
                }

                float raPerpDotN = Vector2.Dot(raPerp, normal);
                float rbPerpDotN = Vector2.Dot(rbPerp, normal);

                float denom = bodyA.InvMass + bodyB.InvMass +
                    (raPerpDotN * raPerpDotN) * bodyA.InvInertia +
                    (rbPerpDotN * rbPerpDotN) * bodyB.InvInertia;

                float j = -(1f + e) * contactVelocityMag;
                j /= denom;
                j /= (float)contactCount;

                Vector2 impulse = j * normal;
                impulseList[i] = impulse;
            }

            for (int i = 0; i < contactCount; i++)
            {
                Vector2 impulse = impulseList[i];
                Vector2 ra = raList[i];
                Vector2 rb = rbList[i];

                bodyA.LinearVelocity += -impulse * bodyA.InvMass;
                bodyA.AngularVelocity += -PhysicsMath.Cross(ra, impulse) * bodyA.InvInertia;
                bodyB.LinearVelocity += impulse * bodyB.InvMass;
                bodyB.AngularVelocity += PhysicsMath.Cross(rb, impulse) * bodyB.InvInertia;
            }
        }

        private void ResolveCollisionWithRotationAndFricton(in Manifold manifold)
        {
            RigidBody bodyA = manifold.BodyA;
            RigidBody bodyB = manifold.BodyB;
            Vector2 normal = manifold.Normal;
            Vector2 contact1 = manifold.Contact1;
            Vector2 contact2 = manifold.Contact2;
            int contactCount = manifold.ContactCount;

            float sf = (bodyA.StaticFriction + bodyB.StaticFriction) * 0.5f;
            float df = (bodyB.DynamicFriction + bodyA.DynamicFriction) * 0.5f;

            float e = MathF.Min(bodyA.Restituition, bodyB.Restituition);

            this.contactList[0] = contact1;
            this.contactList[1] = contact2;

            for (int i = 0; i < contactCount; i++)
            {
                this.impulseList[i] = Vector2.Zero;
                this.raList[i] = Vector2.Zero;
                this.rbList[i] = Vector2.Zero;
                this.frictionImpulseList[i] = Vector2.Zero;
                this.jList[i] = 0f;
            }

            for (int i = 0; i < contactCount; i++)
            {
                Vector2 ra = contactList[i] - bodyA.Position;
                Vector2 rb = contactList[i] - bodyB.Position;

                raList[i] = ra;
                rbList[i] = rb;

                Vector2 raPerp = new Vector2(-ra.Y, ra.X);
                Vector2 rbPerp = new Vector2(-rb.Y, rb.X);

                Vector2 angularLinearVelocityA = raPerp * bodyA.AngularVelocity;
                Vector2 angularLinearVelocityB = rbPerp * bodyB.AngularVelocity;

                Vector2 relativeVelocity =
                    (bodyB.LinearVelocity + angularLinearVelocityB) -
                    (bodyA.LinearVelocity + angularLinearVelocityA);

                float contactVelocityMag = Vector2.Dot(relativeVelocity, normal);

                if (contactVelocityMag > 0f)
                {
                    continue;
                }

                float raPerpDotN = Vector2.Dot(raPerp, normal);
                float rbPerpDotN = Vector2.Dot(rbPerp, normal);

                float denom = bodyA.InvMass + bodyB.InvMass +
                    (raPerpDotN * raPerpDotN) * bodyA.InvInertia +
                    (rbPerpDotN * rbPerpDotN) * bodyB.InvInertia;

                float j = -(1f + e) * contactVelocityMag;
                j /= denom;
                j /= (float)contactCount;

                jList[i] = j;

                Vector2 impulse = j * normal;
                impulseList[i] = impulse;
            }

            for (int i = 0; i < contactCount; i++)
            {
                Vector2 impulse = impulseList[i];
                Vector2 ra = raList[i];
                Vector2 rb = rbList[i];

                bodyA.LinearVelocity += -impulse * bodyA.InvMass;
                bodyA.AngularVelocity += -PhysicsMath.Cross(ra, impulse) * bodyA.InvInertia;
                bodyB.LinearVelocity += impulse * bodyB.InvMass;
                bodyB.AngularVelocity += PhysicsMath.Cross(rb, impulse) * bodyB.InvInertia;
            }


            for (int i = 0; i < contactCount; i++)
            {
                Vector2 ra = contactList[i] - bodyA.Position;
                Vector2 rb = contactList[i] - bodyB.Position;

                raList[i] = ra;
                rbList[i] = rb;

                Vector2 raPerp = new Vector2(-ra.Y, ra.X);
                Vector2 rbPerp = new Vector2(-rb.Y, rb.X);

                Vector2 angularLinearVelocityA = raPerp * bodyA.AngularVelocity;
                Vector2 angularLinearVelocityB = rbPerp * bodyB.AngularVelocity;

                Vector2 relativeVelocity =
                    (bodyB.LinearVelocity + angularLinearVelocityB) -
                    (bodyA.LinearVelocity + angularLinearVelocityA);

                Vector2 tangent = relativeVelocity - Vector2.Dot(relativeVelocity, normal) * normal;

                if (PhysicsMath.NearlyEqual(tangent, Vector2.Zero)) { continue; }
                else { tangent.Normalize(); }

                float raPerpDotT = Vector2.Dot(raPerp, tangent);
                float rbPerpDotT = Vector2.Dot(rbPerp, tangent);

                float denom = bodyA.InvMass + bodyB.InvMass +
                    (raPerpDotT * raPerpDotT) * bodyA.InvInertia +
                    (rbPerpDotT * rbPerpDotT) * bodyB.InvInertia;

                float jt = -(1f + e) * Vector2.Dot(relativeVelocity, tangent);
                jt /= denom;
                jt /= (float)contactCount;

                Vector2 frictionImpulse;
                float j = jList[i];

                if (MathF.Abs(jt) <= jt * sf)
                {
                    frictionImpulse = jt * tangent;
                }
                else
                {
                    frictionImpulse = -j * tangent * df;
                }

                frictionImpulse = jt * tangent;
                frictionImpulseList[i] = frictionImpulse;
            }

            for (int i = 0; i < contactCount; i++)
            {
                Vector2 frictionImpulse = frictionImpulseList[i];
                Vector2 ra = raList[i];
                Vector2 rb = rbList[i];

                bodyA.LinearVelocity += -frictionImpulse * bodyA.InvMass;
                bodyA.AngularVelocity += -PhysicsMath.Cross(ra, frictionImpulse) * bodyA.InvInertia;
                bodyB.LinearVelocity += frictionImpulse * bodyB.InvMass;
                bodyB.AngularVelocity += PhysicsMath.Cross(rb, frictionImpulse) * bodyB.InvInertia;
            }
        }

    }
}
