using Microsoft.Xna.Framework;
using MonoGame.Extended.Serialization;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace test.Physics
{
    public enum ShapeType
    {
        Circle = 0,
        Box = 1
    }


    public sealed class Rigidbody
    {
        private Vector2 position;
        private Vector2 linearVelocity;
        private float rotation;
        private float rotationalVelocity;

        //Body properties
        public readonly float Density;
        public readonly float Mass;
        public readonly float Restituition;
        public readonly float Area;

        public readonly bool IsStatic;

        public readonly float Radius;
        public readonly float Width;
        public readonly float Height;

        public readonly ShapeType ShapeType;

        public Vector2 Position
        {
            get { return this.position; }
        }

        private Rigidbody(Vector2 position, float density, float mass, float restituition, float area,
            bool isStatic, float radius, float width, float height, ShapeType shapeType)
        {
            this.position = position;
            this.linearVelocity = Vector2.Zero;
            this.rotation = 0f;
            this.rotationalVelocity = 0f;

            this.Density = density;
            this.Mass = mass;
            this.Restituition = restituition;
            this.Area = area;

            this.IsStatic = isStatic;

            this.Radius = radius;
            this.Width = width;
            this.Height = height;

            this.ShapeType = shapeType;
        }

        public static bool CreateCircle(float radius, Vector2 position, float density, bool isStatic, float restituition, out Rigidbody body, out string errorMsg)
        {
            body = null;
            errorMsg = String.Empty;

            float area = radius * radius * MathF.PI;

            if(area < World.MinBodySize)
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
            float mass = area * density;

            body = new Rigidbody(position, density, mass, restituition, area, isStatic, radius, 0, 0, ShapeType.Circle);
            return true;
        }

        public static bool CreateBox(float width, float height, Vector2 position, float density, bool isStatic, float restituition, out Rigidbody body, out string errorMsg)
        {
            body = null;
            errorMsg = String.Empty;

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
            float mass = area * density;

            body = new Rigidbody(position, density, mass, restituition, area, isStatic, 0, width, height, ShapeType.Box);
            return true;
        }

    }
}
