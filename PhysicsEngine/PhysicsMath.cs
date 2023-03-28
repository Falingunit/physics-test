using System;
using Microsoft.Xna.Framework;

namespace PhysicsEngine
{
    public static class PhysicsMath
    {
        public static Vector2 Transform(Vector2 v, Transform transform)
        {
            return new Vector2(transform.cos * v.X - transform.sin * v.Y + transform.x,
                               transform.sin * v.X + transform.cos * v.Y + transform.y);
        }
        public static bool NearlyEqual(float a, float b)
        {
            return MathF.Abs(a - b) < World.VerySmallAmount;
        }

        public static bool NearlyEqual(Vector2 a, Vector2 b)
        {
            return Vector2.DistanceSquared(a, b) < World.VerySmallAmount;
        }
        public static float Cross(Vector2 value1, Vector2 value2)
        {
            return value1.X * value2.Y - value1.Y * value2.X;
        }

    }
}
