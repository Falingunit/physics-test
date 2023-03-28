using Microsoft.Xna.Framework;

namespace PhysicsEngine
{
    public class Constraint
    {
        private readonly RigidBody bodyA;
        private readonly RigidBody bodyB;
        private readonly Vector2 constraintCenter;
        private readonly float constraintMax;
        private readonly float constraintMin;
        private readonly float maxVelocity;
        private readonly float maxVelocitySq;
        private readonly bool isStatic;

        internal Constraint(RigidBody bodyA, RigidBody bodyB, Vector2 constraintCenter, float constraintMax, float constraintMin, float maxVelocity, bool isStatic)
        {
            this.bodyA = bodyA;
            this.bodyB = bodyB;
            this.constraintCenter = constraintCenter;
            this.constraintMax = constraintMax;
            this.constraintMin = constraintMin;
            this.maxVelocity = maxVelocity;
            this.maxVelocitySq = maxVelocity * maxVelocity;
            this.isStatic = isStatic;
        }

        public void Update()
        {
            Vector2 ab = Vector2.Zero;

            if (isStatic)
            {
                if (bodyB.LinearVelocity.LengthSquared() <= maxVelocitySq)
                {
                    ab = bodyB.Position - constraintCenter;

                    if (ab.Length() > constraintMax)
                    {
                        float depth = ab.Length() - constraintMax;
                        ab.Normalize();
                        this.bodyB.Move(-depth * ab);
                        this.bodyB.LinearVelocity += ab * -depth * bodyB.InvMass;
                    }
                    else if (ab.Length() < constraintMin)
                    {
                        float depth = constraintMin - ab.Length();
                        ab.Normalize();
                        this.bodyB.Move(depth * ab);
                        this.bodyB.LinearVelocity += ab * depth * bodyB.InvMass;
                    }
                }
            }
            else
            {
                if ((bodyB.LinearVelocity - bodyA.LinearVelocity).LengthSquared() <= maxVelocitySq)
                ab = bodyB.Position - bodyA.Position;

                if (ab.LengthSquared() > constraintMax * constraintMax)
                {
                    float depth = ab.Length() - constraintMax;
                    ab.Normalize();
                    this.bodyA.Move(-depth * ab / 2f);
                    this.bodyA.LinearVelocity += ab * -depth * bodyB.InvMass / 2f;

                    this.bodyB.Move(depth * ab / 2f);
                    this.bodyB.LinearVelocity += ab * depth * bodyB.InvMass / 2f;

                }
                else if (ab.LengthSquared() < constraintMin * constraintMin)
                {
                    float depth = constraintMin - ab.Length();
                    ab.Normalize();
                    this.bodyA.Move(-depth * ab / 2f);
                    this.bodyA.LinearVelocity += ab * -depth * bodyB.InvMass / 2f;

                    this.bodyB.Move(depth * ab / 2f);
                    this.bodyB.LinearVelocity += ab * -depth * bodyB.InvMass / 2f;

                }
            }
        }
    }
}