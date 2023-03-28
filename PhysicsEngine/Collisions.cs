using Microsoft.Xna.Framework;
using System;
using System.Runtime.Intrinsics;
using System.Xml.Serialization;

namespace PhysicsEngine
{
    public static class Collisions
    {
        public static bool IntersectAABBs(AABB aabbA, AABB aabbB)
        {
            if(aabbA.Max.X < aabbB.Min.X || aabbB.Max.X < aabbA.Min.X ||
                aabbA.Max.Y < aabbB.Min.Y || aabbB.Max.Y < aabbA.Min.Y)
            {
                return false;
            }

            return true;
        }

        public static bool IntersectCirclePolygon(
            Vector2 circleCenter, float circleRadius, Vector2 polygonCenter,
            Vector2[] polygonVertices, out Vector2 normal, out float depth)
        {
            normal = Vector2.Zero;
            depth = float.MaxValue;

            Vector2 axis = Vector2.Zero;
            float axisDepth = 0f;
            float minA, maxA, minB, maxB;

            for (int i = 0; i < polygonVertices.Length; i++)
            {
                Vector2 va = polygonVertices[i];
                Vector2 vb = polygonVertices[(i + 1) % polygonVertices.Length];

                Vector2 edge = vb - va;
                axis = new(-edge.Y, edge.X);
                axis.Normalize();

                ProjectVertices(polygonVertices, axis, out minA, out maxA);
                ProjectCircle(circleCenter, circleRadius, axis, out minB, out maxB);

                if (minA >= maxB || minB >= maxA)
                {
                    return false;
                }

                axisDepth = MathF.Min(maxB - minA, maxA - minB);

                if (axisDepth < depth)
                {
                    depth = axisDepth;
                    normal = axis;
                }

                int cpIndex = FindClosestPointOnPolygon(circleCenter, polygonVertices);
                Vector2 cp = polygonVertices[cpIndex];


                axis = cp - circleCenter;
                axis.Normalize();


                ProjectVertices(polygonVertices, axis, out minA, out maxA);
                ProjectCircle(circleCenter, circleRadius, axis, out minB, out maxB);

                if (minA >= maxB || minB >= maxA)
                {
                    return false;
                }

                axisDepth = MathF.Min(maxB - minA, maxA - minB);

                if (axisDepth < depth)
                {
                    depth = axisDepth;
                    normal = axis;
                }
            }

            Vector2 direction = polygonCenter - circleCenter;

            if (Vector2.Dot(direction, normal) < 0f)
            {
                normal = -normal;
            }


            return true;
        }

        public static bool IntersectPolygons(
            Vector2 centerA, Vector2[] verticesA, Vector2 centerB,
            Vector2[] verticesB, out Vector2 normal, out float depth)
        {
            normal = Vector2.Zero;
            depth = float.MaxValue;

            for (int i = 0; i < verticesA.Length; i++)
            {
                Vector2 va = verticesA[i];
                Vector2 vb = verticesA[(i + 1) % verticesA.Length];

                Vector2 edge = vb - va;
                Vector2 axis = new(-edge.Y, edge.X);
                axis.Normalize();

                ProjectVertices(verticesA, axis, out float minA, out float maxA);
                ProjectVertices(verticesB, axis, out float minB, out float maxB);

                if (minA >= maxB || minB >= maxA)
                {
                    return false;
                }

                float axisDepth = MathF.Min(maxB - minA, maxA - minB);

                if (axisDepth < depth)
                {
                    depth = axisDepth;
                    normal = axis;
                }
            }

            for (int i = 0; i < verticesB.Length; i++)
            {
                Vector2 va = verticesB[i];
                Vector2 vb = verticesB[(i + 1) % verticesB.Length];

                Vector2 edge = vb - va;
                Vector2 axis = new(-edge.Y, edge.X);
                axis.Normalize();

                ProjectVertices(verticesA, axis, out float minA, out float maxA);
                ProjectVertices(verticesB, axis, out float minB, out float maxB);

                if (minA >= maxB || minB >= maxA)
                {
                    return false;
                }

                float axisDepth = MathF.Min(maxB - minA, maxA - minB);

                if (axisDepth < depth)
                {
                    depth = axisDepth;
                    normal = axis;
                }
            }

            Vector2 direction = centerB - centerA;

            if (Vector2.Dot(direction, normal) < 0f)
            {
                normal = -normal;
            }

            return true;
        }

        public static bool IntersectCicles(Vector2 centerA, float radiusA, Vector2 centerB, float radiusB,
            out Vector2 normal, out float depth)
        {
            normal = Vector2.Zero;
            depth = 0;

            float distance = Vector2.Distance(centerA, centerB);
            float radii = radiusA + radiusB;



            if (distance >= radii)
            {
                return false;
            }

            normal = Vector2.Normalize(centerB - centerA);
            depth = radii - distance;

            return true;
        }

        public static bool Collide(RigidBody bodyA, RigidBody bodyB, out Vector2 normal, out float depth)
        {
            normal = Vector2.Zero;
            depth = 0f;

            ShapeType shapeTypeA = bodyA.ShapeType;
            ShapeType shapetypeB = bodyB.ShapeType;

            if (shapeTypeA is ShapeType.Box)
            {
                if (shapetypeB is ShapeType.Box)
                {
                    return Collisions.IntersectPolygons(
                        bodyA.Position, bodyA.GetTransformedVertices(), bodyB.Position,
                        bodyB.GetTransformedVertices(),
                        out normal, out depth);
                }
                else if (shapetypeB == ShapeType.Circle)
                {
                    bool result = Collisions.IntersectCirclePolygon(
                        bodyB.Position, bodyB.Radius, bodyA.Position, bodyA.GetTransformedVertices(),
                        out normal, out depth);
                    normal = -normal;
                    return result;
                }
            }
            else if (shapeTypeA is ShapeType.Circle)
            {
                if (shapetypeB is ShapeType.Box)
                {
                    return Collisions.IntersectCirclePolygon(
                        bodyA.Position, bodyA.Radius, bodyB.Position, bodyB.GetTransformedVertices(),
                        out normal, out depth);
                }
                else if (shapetypeB == ShapeType.Circle)
                {
                    return Collisions.IntersectCicles(
                        bodyA.Position, bodyA.Radius, bodyB.Position, bodyB.Radius, out normal,
                        out depth);
                }
            }

            return false;
        }

        public static void FindContactPoints(RigidBody bodyA, RigidBody bodyB, out Vector2 contact1,
            out Vector2 contact2, out int contactCount)
        {
            contact1 = Vector2.Zero;
            contact2 = Vector2.Zero;
            contactCount = 0;

            ShapeType shapeTypeA = bodyA.ShapeType;
            ShapeType shapetypeB = bodyB.ShapeType;

            if (shapeTypeA is ShapeType.Box)
            {
                if (shapetypeB is ShapeType.Box)
                {
                    Collisions.FindPolygonPolygonContactPoints(
                        bodyA.GetTransformedVertices(), bodyB.GetTransformedVertices(),
                        out contact1, out contact2, out contactCount);
                }
                else if (shapetypeB == ShapeType.Circle)
                {
                    Collisions.FindCirclePolygonContactPoint(
                        bodyB.Position, bodyB.Radius, bodyA.Position, bodyA.GetTransformedVertices(), out contact1);
                    contactCount = 1;
                }
            }
            else if (shapeTypeA is ShapeType.Circle)
            {
                if (shapetypeB is ShapeType.Box)
                {
                    Collisions.FindCirclePolygonContactPoint(
                        bodyA.Position, bodyA.Radius, bodyB.Position, bodyB.GetTransformedVertices(), out contact1);
                    contactCount = 1;
                }
                else if (shapetypeB == ShapeType.Circle)
                {
                    Collisions.FindCircleCircleContactPoint(
                        bodyA.Position, bodyA.Radius, bodyB.Position, out contact1);
                    contactCount = 1;
                }
            }
        }

        private static void FindPolygonPolygonContactPoints(
            Vector2[] verticesA, Vector2[] verticesB, out Vector2 contact1,
            out Vector2 contact2, out int contactCount)
        {
            contact1 = Vector2.Zero;
            contact2 = Vector2.Zero;
            contactCount = 0;

            float minDistSq = float.MaxValue;

            for (int i = 0; i < verticesA.Length; i++)
            {
                Vector2 p = verticesA[i];

                for (int j = 0;  j < verticesB.Length; j++)
                {
                    Vector2 va = verticesB[j];
                    Vector2 vb = verticesB[(j + 1) % verticesB.Length];

                    Collisions.PointSegmentDistance(p, va, vb, out float distSq, out Vector2 cp);

                    if(PhysicsMath.NearlyEqual(distSq, minDistSq))
                    {
                        if (!PhysicsMath.NearlyEqual(cp, contact1))
                        {
                            contact2 = cp;
                            contactCount = 2;
                        }
                    }
                    else if(distSq < minDistSq)
                    {
                        minDistSq = distSq;
                        contactCount = 1;
                        contact1 = cp;
                    }
                }
            }

            for (int i = 0; i < verticesA.Length; i++)
            {
                Vector2 p = verticesB[i];

                for (int j = 0; j < verticesA.Length; j++)
                {
                    Vector2 va = verticesA[j];
                    Vector2 vb = verticesA[(j + 1) % verticesA.Length];

                    Collisions.PointSegmentDistance(p, va, vb, out float distSq, out Vector2 cp);

                    if (PhysicsMath.NearlyEqual(distSq, minDistSq))
                    {
                        if (!PhysicsMath.NearlyEqual(cp, contact1))
                        {
                            contact2 = cp;
                            contactCount = 2;
                        }
                    }
                    else if (distSq < minDistSq)
                    {
                        minDistSq = distSq;
                        contactCount = 1;
                        contact1 = cp;
                    }
                }
            }
        }

        private static void FindCirclePolygonContactPoint(
            Vector2 circleCenter, float circleRadius, Vector2 polygonCenter,
            Vector2[] polygonVertices, out Vector2 contactPoint)
        {
            contactPoint = Vector2.Zero;
            float minDistSq = float.MaxValue;

            for(int i = 0; i < polygonVertices.Length; i++)
            {
                Vector2 va = polygonVertices[i];
                Vector2 vb = polygonVertices[(i + 1) % polygonVertices.Length];

                Collisions.PointSegmentDistance(circleCenter, va, vb, out float distanceSquared, out Vector2 contact);

                if (distanceSquared < minDistSq)
                {
                    minDistSq = distanceSquared;
                    contactPoint = contact;
                }
            }
        }

        private static void FindCircleCircleContactPoint(
            Vector2 centerA, float radiusA, Vector2 centerB, out Vector2 contactPoint)
        {
            Vector2 ab = centerB - centerA;
            Vector2 dir = Vector2.Normalize(ab);
            contactPoint = centerA + dir * radiusA;
        }

        private static int FindClosestPointOnPolygon(Vector2 checkPoint, Vector2[] vertices)
        {
            int result = -1;
            float minDistance = float.MaxValue;

            for (int i = 0; i < vertices.Length; i++)
            {
                Vector2 v = vertices[i];
                float distance = Vector2.Distance(v, checkPoint);

                if (distance < minDistance)
                {
                    minDistance = distance;
                    result = i;
                }

            }

            return result;
        }

        private static void ProjectVertices(Vector2[] vertices, Vector2 axis, out float min, out float max)
        {
            min = float.MaxValue;
            max = float.MinValue;

            for (int i = 0; i < vertices.Length; i++)
            {
                Vector2 v = vertices[i];
                float proj = Vector2.Dot(v, axis);

                if (proj < min) { min = proj; }
                if (proj > max) { max = proj; }
            }
        }

        private static void ProjectCircle(Vector2 center, float radius, Vector2 axis, out float min, out float max)
        {
            Vector2 direction = Vector2.Normalize(axis);
            Vector2 directionAndRadius = direction * radius;

            Vector2 point1 = center + directionAndRadius;
            Vector2 point2 = center - directionAndRadius;

            min = Vector2.Dot(point1, axis);
            max = Vector2.Dot(point2, axis);

            if (min > max)
            {
                (min, max) = (max, min);
            }
        }

        public static void PointSegmentDistance(
            Vector2 p,  Vector2 a, Vector2 b, out float distanceSquared, out Vector2 closestPoint)
        {
            Vector2 ab = b - a;
            Vector2 ap = p - a;

            float proj = Vector2.Dot(ap, ab);
            float abLenSq = ab.LengthSquared();
            float d = proj / abLenSq;

            if (d <= 0f)
            {
                closestPoint = a;
            }
            else if (d >= 1f)
            {
                closestPoint = b;
            }
            else
            {
                closestPoint = a + ab * d;
            }

            distanceSquared = Vector2.DistanceSquared(p, closestPoint);
        }
    }
}
