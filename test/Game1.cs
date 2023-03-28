using Flat;
using Flat.Graphics;
using Flat.Input;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using PhysicsEngine;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Net.Http.Headers;
using System.Xml;

namespace test
{
    public class Game1 : Game
    {

        private GraphicsDeviceManager _graphics;
        private Screen screen;
        private Sprites sprites;
        private Shapes shapes;
        private Camera camera;
        private SpriteFont fontConsolas18;

        private World world;

        private Vector2 point;
        private RigidBody body;
        private List<Entity> entities;
        private List<Entity> entitiesRemovalList;

        private Stopwatch watch;

        private int totalBodyCount = 0;
        private double totalWorldStepTime = 0d;
        private int totalSampleCount = 0;
        private Stopwatch sampleTimer = new Stopwatch();

        private string worldStepTimeString = string.Empty;
        private string bodyCountString = string.Empty;

        public static int FPS;
        private TimeSpan counterElapsed = TimeSpan.Zero;
        private int fpsCounter = 0;

        public Game1()
        {
            _graphics = new GraphicsDeviceManager(this);
            this._graphics.SynchronizeWithVerticalRetrace = true;
            this._graphics.PreferredBackBufferWidth = 1280;
            this._graphics.PreferredBackBufferHeight = 720;
            Content.RootDirectory = "Content";
            IsMouseVisible = true;
        }

        protected override void Initialize()
        {
            this.Window.Title = "Extraterrestrial physics test | FPS:--- | Memory Usage:---";
            this.Window.AllowUserResizing = true;

            FlatUtil.SetRelativeBackBufferSize(this._graphics, 0.9f);

            //_graphics.IsFullScreen = true;
            //_graphics.ApplyChanges();

            this.screen = new Screen(this, 1280, 768);
            this.sprites = new Sprites(this);
            this.shapes = new Shapes(this);
            this.camera = new Camera(this.screen);
            this.camera.Zoom = 20;

            this.camera.GetExtents(out float left,out float right, out float bottom, out float top);      

            world = new World();

            this.entities = new List<Entity>();
            this.entitiesRemovalList = new List<Entity>();

            float padding = MathF.Abs(right - left) * 0.10f;

            point = new Vector2(right / 2f, top / 2f);
            Entity entity = new Entity(world, 0.5f, false, new Vector2(right / 2f + 0.5f, top / 2f + 1f));
            body = entity.Body;
            Constraint constraint = world.CreateStaticConstraint(point, body, 10f, 1f, 100f);

            entities.Add(entity);

            this.watch = new Stopwatch();
            this.sampleTimer.Start();

            base.Initialize();
        }

        protected override void LoadContent()
        {
            this.fontConsolas18 = this.Content.Load<SpriteFont>("Consolas18");
        }

        protected override void Update(GameTime gameTime)
        {
            FlatKeyboard keyboard = FlatKeyboard.Instance;
            FlatMouse mouse = FlatMouse.Instance;

            keyboard.Update();
            mouse.Update();

            //if (mouse.IsLeftMouseButtonPressed())
            //{
            //    float width = RandomHelper.RandomSingle(2f, 3f);
            //    float height = RandomHelper.RandomSingle(2f, 3f);

            //    if (!RigidBody.CreateBox(width, height, 2, false, 0.6f, 0.6f, out RigidBody body, out string errorMsg))
            //    {
            //        throw new Exception(errorMsg);
            //    }

            //    body.MoveTo(mouse.GetMouseWorldPosition(this, this.screen, this.camera));
            //    this.world.AddBody(body);
            //    entities.Add(new(body));
            //}

            if (mouse.IsRightMouseButtonPressed())
            {
                float radius = RandomHelper.RandomSingle(1f, 1.25f);

                if (!RigidBody.CreateCircle(radius, 2, false, 0.6f, 0.6f, out RigidBody body, out string errorMsg))
                {
                    throw new Exception(errorMsg);
                }

                body.MoveTo(mouse.GetMouseWorldPosition(this, this.screen, this.camera));
                this.world.AddBody(body);
                entities.Add(new(body));
            }

            if (mouse.IsLeftMouseButtonPressed())
            {
                body.MoveTo(mouse.GetMouseWorldPosition(this, this.screen, this.camera));
            }


            if (this.sampleTimer.Elapsed.TotalSeconds > 1d)
            {
                this.bodyCountString = "Body count: " + Math.Round(this.totalBodyCount / (double)this.totalSampleCount, 0).ToString();
                this.worldStepTimeString = "World step time: " + Math.Round(this.totalWorldStepTime / (double)this.totalSampleCount, 4).ToString();
                this.totalBodyCount = 0;
                this.totalSampleCount = 0;
                this.totalWorldStepTime = 0d;
                this.sampleTimer.Restart();
            }

            this.watch.Restart();
            this.world.Step((float)gameTime.ElapsedGameTime.TotalSeconds, 20);
            this.watch.Stop();

            this.totalWorldStepTime += this.watch.Elapsed.TotalMilliseconds;
            this.totalBodyCount += this.world.BodyCount;
            this.totalSampleCount++;

            this.camera.GetExtents(out _, out _, out float bottom, out _);

            this.entitiesRemovalList.Clear();

            for (int i = 0; i < this.entities.Count; i++)
            {
                Entity entity = entities[i];
                RigidBody body = entities[i].Body;
                if (body.IsStatic) continue;

                AABB box = body.GetAABB();

                if (box.Max.Y < bottom)
                {
                    this.entitiesRemovalList.Add(entity);
                }

            }

            for(int i = 0; i < entitiesRemovalList.Count; i++)
            {
                Entity entity = entitiesRemovalList[i];
                RigidBody body = entity.Body;
                this.world.RemoveBody(body);
                entities.Remove(entity);
            }

            base.Update(gameTime);
        }

        protected override void Draw(GameTime gameTime)
        {
            this.screen.Set();
            GraphicsDevice.Clear(new Color(40, 40, 40));

            this.shapes.Begin(this.camera);

            for (int i = 0; i < this.entities.Count; i++)
            {
                entities[i].Draw(this.shapes);
            }

            shapes.DrawCircle(point, 10f, 26, Color.White);
            shapes.DrawCircle(point, 1f, 20, Color.White);
            shapes.DrawCircle(point, 0.075f, 2, Color.White);

            this.shapes.End();
            
            Vector2 stringSize = this.fontConsolas18.MeasureString(this.bodyCountString);

            this.sprites.Begin();
            this.sprites.DrawString(this.fontConsolas18, this.bodyCountString, new(0, 0), Color.White);
            this.sprites.DrawString(this.fontConsolas18, this.worldStepTimeString, new(0, stringSize.Y), Color.White);
            this.sprites.End();

            this.screen.Unset();
            this.screen.Present(this.sprites);

            base.Draw(gameTime);

            fpsCounter++;
            counterElapsed += gameTime.ElapsedGameTime;
            if (counterElapsed >= TimeSpan.FromSeconds(1))
            {
                Window.Title = "Extraterrestrial physics test" + " | FPS: " + fpsCounter.ToString() + " | Memory usage: " + (GC.GetTotalMemory(false) / 1048576f).ToString("F") + " MB";

                FPS = fpsCounter;
                fpsCounter = 0;
                counterElapsed -= TimeSpan.FromSeconds(1);
            }
        }
    }
}