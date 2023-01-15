using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using MonoGame.Extended;
using System;
using System.Collections.Generic;
using test.Physics;

namespace test
{
    public class Game1 : Game
    {
        private GraphicsDeviceManager _graphics;
        private SpriteBatch _spriteBatch;
        private Random random;

        private List<Rigidbody> bodyList;

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
            Window.Title = "Extraterrestrial physics test | FPS:--- | Memory Usage:---";

            random = new Random();

            this.bodyList = new List<Rigidbody>();
            int bodyCount = 15;

            for(int i = 0; i < bodyCount; i++)
            {
                int type = random.Next(2);

                Rigidbody body = null;

                float x = random.Next(40, 1260-20);
                float y = random.Next(40, 700-20);

                if (type == (int)ShapeType.Circle)
                {
                    if(!Rigidbody.CreateCircle(30f, new Vector2(x, y), 2f, false, 0.5f, out body, out string errmsg))
                    {
                        throw new Exception(errmsg);
                    }

                }
                else if(type == (int)ShapeType.Box)
                {
                    if (!Rigidbody.CreateBox(60f, 60f, new Vector2(x, y), 2f, false, 0.5f, out body, out string errmsg))
                    {
                        throw new Exception(errmsg);
                    }
                }

                bodyList.Add(body);
            }

            base.Initialize();
        }

        protected override void LoadContent()
        {
            _spriteBatch = new SpriteBatch(GraphicsDevice);

            // TODO: use this.Content to load your game content here
        }

        protected override void Update(GameTime gameTime)
        {
            if (GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed || Keyboard.GetState().IsKeyDown(Keys.Escape))
                Exit();

            base.Update(gameTime);
        }

        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(Color.DarkBlue);

            _spriteBatch.Begin();

            foreach(var body in bodyList)
            {
                Vector2 pos = body.Position;
                if(body.ShapeType == ShapeType.Circle)
                {
                    _spriteBatch.DrawCircle(pos, body.Radius, 26, Color.White);
                }
                else if(body.ShapeType == ShapeType.Box)
                {
                    _spriteBatch.DrawRectangle(pos.X, pos.Y, body.Width, body.Height, Color.Red);
                }
            }

            _spriteBatch.End();

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