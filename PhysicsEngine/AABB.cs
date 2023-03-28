using Microsoft.Xna.Framework;

namespace PhysicsEngine
{
    public readonly struct AABB
    {
        public readonly Vector2 Min, Max;

        public AABB(Vector2 min, Vector2 max)
        {
            Max = max;
            Min = min;
        }

        public AABB(float minX, float minY, float maxX, float maxY)
        {
            Max = new(maxX, maxY);
            Min = new(minX, minY);
        }

    }
}
