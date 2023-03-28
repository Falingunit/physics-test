using System;
using Microsoft.Xna.Framework;

namespace PhysicsEngine
{
    public readonly struct Transform
    {
        public readonly float x;
        public readonly float y;
        public readonly float sin;
        public readonly float cos;

        public readonly static Transform Zero = new Transform(0f, 0f, 0f);

        public Transform(Vector2 position, float angle)
        {
            x = position.X;
            y = position.Y;
            sin = MathF.Sin(angle);
            cos = MathF.Cos(angle);
        }

        public Transform(float x, float y, float angle)
        {
            this.x = x;
            this.y = y;
            sin = MathF.Sin(angle);
            cos = MathF.Cos(angle);
        }

    }
}
