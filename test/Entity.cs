using Flat;
using Flat.Graphics;
using Microsoft.Xna.Framework;
using PhysicsEngine;
using System;

namespace test
{
    public sealed class Entity
    {
        public readonly RigidBody Body;
        public readonly Color Color;
        public readonly Color OutlineColor;

        public Entity(RigidBody body, Color color, Color outlineColor)
        {
            this.Body = body;
            this.Color = color;
            this.OutlineColor = outlineColor;
        }

        public Entity(RigidBody body)
        {
            this.Body = body;
            this.Color = RandomHelper.RandomColor();
            this.OutlineColor = Color.White;
        }

        public Entity(World world, float radius, bool isStatic, Vector2 position)
        {
            if(!RigidBody.CreateCircle(radius, 1f, isStatic, 0.5f, 0.6f, out RigidBody body, out string errMsg))
            {
                throw new Exception(errMsg);
            }

            body.MoveTo(position);
            this.Body = body;
            world.AddBody(body);
            this.Color = RandomHelper.RandomColor();
            this.OutlineColor = Color.White;
        }

        public Entity(World world, float width, float height, bool isStatic, Vector2 position)
        {
            if (!RigidBody.CreateBox(width, height, 1f, isStatic, 0.5f, 0.6f, out RigidBody body, out string errMsg))
            {
                throw new Exception(errMsg);
            }

            body.MoveTo(position);
            this.Body = body;
            world.AddBody(body);
            this.Color = RandomHelper.RandomColor();
            this.OutlineColor = Color.White;
        }

        public Entity(World world, float radius, bool isStatic, Vector2 position, Color color, Color outlineColor)
        {
            if (!RigidBody.CreateCircle(radius, 1f, isStatic, 0.5f, 0.6f, out RigidBody body, out string errMsg))
            {
                throw new Exception(errMsg);
            }

            body.MoveTo(position);
            this.Body = body;
            world.AddBody(body);
            this.Color = color;
            this.OutlineColor = outlineColor;
        }

        public Entity(World world, float width, float height, bool isStatic, Vector2 position, Color color, Color outlineColor)
        {
            if (!RigidBody.CreateBox(width, height, 1f, isStatic, 0.5f, 0.6f, out RigidBody body, out string errMsg))
            {
                throw new Exception(errMsg);
            }

            body.MoveTo(position);
            this.Body = body;
            world.AddBody(body);
            this.Color = color;
            this.OutlineColor = outlineColor;
        }

        public void Draw(Shapes shapes)
        {
            if (this.Body.ShapeType == ShapeType.Circle)
            {
                shapes.DrawCircleFill(this.Body.Position, this.Body.Radius, 26, this.Color);
                shapes.DrawCircle(this.Body.Position, this.Body.Radius, 26, this.OutlineColor);
            }
            else if (this.Body.ShapeType == ShapeType.Box)
            {
                shapes.DrawBoxFill(this.Body.Position, this.Body.Width, this.Body.Height, this.Body.Angle, Color);
                shapes.DrawBox(this.Body.Position, this.Body.Width, this.Body.Height, this.Body.Angle, this.OutlineColor);
            }
        }
    }
}
