using Microsoft.Xna.Framework;

using System;
using System.Diagnostics.Tracing;
using System.Linq.Expressions;
using System.Runtime.InteropServices;

namespace PhysicsEngine
{
    public enum ShapeType
    {
        Circle = 0,
        Box = 1
    }

    public sealed class RigidBody
    {
        private Vector2 position;
        private Vector2 linearVelocity;
        private float angle;
        private float angularVelocity;
        private Vector2 force;

        //Body properties
        public readonly float Density;
        public readonly float Mass;
        public readonly float InvMass;
        public readonly float Restituition;
        public readonly float Area;
        public readonly float Inertia;
        public readonly float InvInertia;
        public readonly bool IsStatic;
        public readonly float Radius;
        public readonly float Width;
        public readonly float Height;
        public readonly float StaticFriction;
        public readonly float DynamicFriction;

        //Caches
        private readonly Vector2[]? vertices;
        private Vector2[]? transformedVertices;
        private AABB aabb;

        //Cache update required?
        private bool transformUpdateRequired;
        private bool aabbUpdateRequired;

        public readonly ShapeType ShapeType;

        public Vector2 Position
        {
            get { return position; }
        }

        public float Angle
        {
            get { return angle; }
        }

        public Vector2 LinearVelocity
        {
            get { return linearVelocity; }
            internal set { linearVelocity = value; }
        }

        public float AngularVelocity
        {
            get { return angularVelocity; }
            internal set { angularVelocity = value; }
        }

        private RigidBody(float density, float mass, float inertia, float restituition, float staticFriction, float dynamicFriction, float area,
            bool isStatic, float radius, float width, float height, Vector2[]? vertices, ShapeType shapeType)
        {
            this.position = Vector2.Zero;
            this.linearVelocity = Vector2.Zero;
            this.angle = 0f;
            this.angularVelocity = 0f;
            this.force = Vector2.Zero;

            this.Density = density;
            this.Mass = mass;
            this.InvMass = mass > 0f ? 1f / mass : 0f;
            this.Inertia = inertia;
            this.InvInertia = inertia > 0f ? 1f / inertia : 0f;
            this.Restituition = restituition;
            this.Area = area;
            this.IsStatic = isStatic;
            this.Radius = radius;
            this.Width = width;
            this.Height = height;
            this.StaticFriction = 0.6f;
            this.DynamicFriction = 0.4f;

            this.ShapeType = shapeType;

            this.vertices = vertices;
            this.transformedVertices = shapeType == ShapeType.Box ? new Vector2[vertices.Length] : null;

            this.aabbUpdateRequired = true; 
            this.transformUpdateRequired = true;
        }

        internal void Step(float dt, Vector2 gravity, int iterations)
        {
            if (IsStatic) { return; }


            dt /= iterations;

            //Vector2 acceleration = force / Mass;
            //this.linearVelocity += acceleration * dt;

            linearVelocity += gravity * dt;
            position += linearVelocity * dt;
            angle += angularVelocity * dt;

            force = Vector2.Zero;
            transformUpdateRequired = true;
            aabbUpdateRequired = true;
        }

        private static Vector2[] CreateBoxVertices(float width, float height)
        {
            float left = -width / 2f;
            float right = left + width;
            float bottom = -height / 2f;
            float top = bottom + height;

            Vector2[] vertices = new Vector2[4];
            vertices[0] = new Vector2(left, top);
            vertices[1] = new Vector2(right, top);
            vertices[2] = new Vector2(right, bottom);
            vertices[3] = new Vector2(left, bottom);

            return vertices;
        }

        public static bool CreateCircle(
            float radius, float density, bool isStatic,
            float restituition, float friction, out RigidBody? body, out string errorMsg)
        {
            body = null;
            errorMsg = string.Empty;

            float area = radius * radius * MathF.PI;

            if (area < World.MinBodySize)
            {
                errorMsg = $"Circle radius is too small. Minimum circle area is {World.MinBodySize}";
                return false;
            }
            else if (area > World.MaxBodySize)
            {
                errorMsg = $"Circle radius is too large. Maximum circle area is {World.MaxBodySize}";
                return false;
            }

            if (density < World.MinDensity)
            {
                errorMsg = $"Density is too small. Minimum density is {World.MinDensity}";
                return false;
            }
            else if (density > World.MaxDensity)
            {
                errorMsg = $"Density is too large. Maximum density is {World.MaxDensity}";
                return false;
            }

            restituition = Math.Clamp(restituition, 0, 1);

            float mass = 0f;
            float inertia = 0f;

            if (!isStatic)
            {
                mass = area * density;
                inertia = (1f / 2) * mass * radius * radius;
            }

            body = new RigidBody(density, mass, inertia, restituition, friction, friction * 0.5f, area, isStatic, radius, 0, 0, null, ShapeType.Circle);
            return true;
        }

        public static bool CreateBox(
            float width, float height, float density, bool isStatic,
            float restituition, float friction, out RigidBody body, out string errorMsg)
        {
            body = null;
            errorMsg = string.Empty;

            float area = width * height;

            if (area < World.MinBodySize)
            {
                errorMsg = $"Area is too small. Minimum box area is {World.MinBodySize}";
                return false;
            }
            else if (area > World.MaxBodySize)
            {
                errorMsg = $"Area is too large. Maximum box area is {World.MaxBodySize}";
                return false;
            }

            if (density < World.MinDensity)
            {
                errorMsg = $"Density is too small. Minimum density is {World.MinDensity}";
                return false;
            }
            else if (density > World.MaxDensity)
            {
                errorMsg = $"Density is too large. Maximum density is {World.MaxDensity}";
                return false;
            }

            restituition = Math.Clamp(restituition, 0, 1);

            float mass = 0f;
            float inertia = 0f;

            if (!isStatic)
            {
                mass = area * density;
                inertia = (1f / 12) * mass * (width * width + height * height);
            }

            Vector2[] vertices = CreateBoxVertices(width, height);

            body = new RigidBody(density, mass, inertia, restituition, friction, friction * 0.5f, area, isStatic, 0, width, height, vertices, ShapeType.Box);
            return true;
        }

        public AABB GetAABB()
        {
            if (aabbUpdateRequired)
            {
                float minX = float.MaxValue;
                float minY = float.MaxValue;
                float maxX = float.MinValue;
                float maxY = float.MinValue;

                if (ShapeType is ShapeType.Box)
                {
                    Vector2[] vertices = GetTransformedVertices();

                    for (int i = 0; i < vertices.Length; i++)
                    {
                        Vector2 v = vertices[i];

                        if (v.X < minX) minX = v.X;
                        if (v.X > maxX) maxX = v.X;
                        if (v.Y > maxY) maxY = v.Y;
                        if (v.Y < minY) minY = v.Y;
                    }
                }
                else if (ShapeType is ShapeType.Circle)
                {
                    minX = position.X - Radius;
                    minY = position.Y - Radius;
                    maxX = position.X + Radius;
                    maxY = position.Y + Radius;
                }
                else
                {
                    throw new Exception("Unknown shape type.");
                }

                aabb = new AABB(minX, minY, maxX, maxY);
            }

            aabbUpdateRequired = false;
            return aabb;
        }

        public Vector2[] GetTransformedVertices()
        {
            if (transformUpdateRequired)
            {
                Transform transform = new Transform(position, angle);

                for (int i = 0; i < vertices.Length; i++)
                {
                    Vector2 v = vertices[i];
                    transformedVertices[i] = PhysicsMath.Transform(v, transform);
                }
            }

            transformUpdateRequired = false;
            return transformedVertices;
        }

        public void Move(Vector2 delta)
        {
            position += delta;
            transformUpdateRequired = true;
            aabbUpdateRequired = true;
        }

        public void MoveTo(Vector2 position)
        {
            this.position = position;
            transformUpdateRequired = true;
            aabbUpdateRequired = true;
        }

        public void Rotate(float angle)
        {
            this.angle += angle;
            transformUpdateRequired = true;
            aabbUpdateRequired = true;
        }

        public void RotateTo(float angle)
        {
            this.angle = angle;
            this.aabbUpdateRequired = true;
            this.transformUpdateRequired = true;
        }

        public void AddForce(Vector2 force)
        {
            this.force = force;
        }

    }
}
