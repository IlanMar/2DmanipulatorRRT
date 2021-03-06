﻿////Ilan Margolin 2019
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Windows.Forms;
using System.Globalization;
using System.Diagnostics;

namespace ManipulatorRRT
{
    public partial class Form1 : Form
    {
        Graphics g;
        Helper Help = new Helper();

        public Form1()
        {
            InitializeComponent();
            pictureBox1.Image = new Bitmap(pictureBox1.Width, pictureBox1.Height);
            g = Graphics.FromImage(pictureBox1.Image);
        }

        RRT Rrt = new RRT();
        PointF Cinit = new PointF(0, 0); PointF Cgoal = new PointF(40, 40); int Nsteps = 50000; int Nextend = 0;
        Obstacle Obs = new Obstacle();
        bool k = false;

        private void button1_Click(object sender, EventArgs e)
        {
            bool checkBox = false;

            if (checkBox1.Checked)
            {
                checkBox = true;
            }
            else checkBox = false;

            System.Diagnostics.Stopwatch sw = new Stopwatch();//время работы ррт
            sw.Start();
            int dopustimueOtklonenia = int.Parse(textBox9.Text, CultureInfo.InvariantCulture.NumberFormat);
            Cgoal.X = float.Parse(textBox2.Text, CultureInfo.InvariantCulture.NumberFormat);
            Cgoal.Y = float.Parse(textBox3.Text, CultureInfo.InvariantCulture.NumberFormat);
            int edgeMaxLenght = int.Parse(textBox4.Text, CultureInfo.InvariantCulture.NumberFormat);
            Nsteps = int.Parse(textBox5.Text, CultureInfo.InvariantCulture.NumberFormat);
            float qincrement = float.Parse(textBox6.Text, CultureInfo.InvariantCulture.NumberFormat);
            k = Rrt.RRTStart(Cinit, Cgoal, Nsteps, Nextend, edgeMaxLenght, Obs.ObsList, qincrement, checkBox, dopustimueOtklonenia);
            textBox1.Text = Rrt.success.ToString();
            paintTree();
            updateRobotPose(Rrt.T[0]);
            textBox1.Invalidate();
            sw.Stop();
            textBox7.Text = (sw.ElapsedMilliseconds / 1000.0).ToString();
            textBox8.Text = (Rrt.T.Count).ToString(); ;
        }

        private void button2_Click(object sender, EventArgs e)
        {
            if (k)
            {
                Rrt.FindbackPath();
                paintPath();
            }
        }

        public void paintPath()
        {
            Pen REDPen = new Pen(Color.Red, 3);
            for (int i = 0; i < Rrt.backPath.Count - 1; i++)// -2 потомучто не хочу рисовать линию то точки 0, 0
            {
                g.DrawLine(REDPen, pictureBox1.Width - pictureBox1.Width / 2 + Rrt.backPath[i].X, pictureBox1.Height - Rrt.backPath[i].Y, pictureBox1.Width - pictureBox1.Width / 2 + Rrt.backPath[i + 1].X, pictureBox1.Height - Rrt.backPath[i + 1].Y);
            }
            pictureBox1.Invalidate();
        }

        int i = 1;

        private void timer1_Tick(object sender, EventArgs e)
        {
            if (Rrt.T.Count > 0)
            {
                int y = Rrt.backPathTT.Count;

                if (i < y)
                {
                    updateRobotPose(Rrt.backPathTT[y - i]);//, Rrt.backPathTT[i+1]);
                    i++;
                }
            }
        }

        public void paintTree()
        {
            List<GraphT> T = Rrt.T;
            int D = 10;
            g.FillEllipse(Brushes.Red, pictureBox1.Width - pictureBox1.Width / 2 + Cgoal.X - D / 2, pictureBox1.Height - Cgoal.Y - D / 2, D, D); //рисуем целевую точку
            Pen blackPen = new Pen(Color.Black, 1);

            for (int i = 1; i < T.Count; i++)
            {
                g.DrawLine(blackPen, pictureBox1.Width - pictureBox1.Width / 2 + T[i].E.p1.X, pictureBox1.Height - T[i].E.p1.Y, pictureBox1.Width - pictureBox1.Width / 2 + T[i].E.p2.X, pictureBox1.Height - T[i].E.p2.Y);
            }
                        
            pictureBox1.Invalidate();
        }

        private void updateRobotPose(GraphT start)// GraphT goal
        {
            ManipulatorConf tempConf = start.V;
            g.Clear(Color.White);
            paintTree();// заного отрисовываем дерево
            paintPath();
            var D = 10;//радиус кружка звеньев
            var D2 = 15;//кружок основания
            var D3 = 20;//радиус колес
            Pen blackPen = new Pen(Color.Blue, 2);
            Pen blackPen2 = new Pen(Color.Black, 6);
            g.FillRectangle(Brushes.Black, (pictureBox1.Width - pictureBox1.Width / 2 - D2 / 2) + tempConf.qP, pictureBox1.Height - D2 / 2 - 19, D2, D2); //рисуем квадрат  основания
            g.FillEllipse(Brushes.Black, (pictureBox1.Width - pictureBox1.Width / 2 - D3 / 2) + tempConf.qP - 30, pictureBox1.Height - D3, D3, D3); //рисуем колесо платформы
            g.FillEllipse(Brushes.Black, (pictureBox1.Width - pictureBox1.Width / 2 - D3 / 2) + tempConf.qP + 30, pictureBox1.Height - D3, D3, D3); //рисуем колесо платформы
            g.DrawLine(blackPen2, pictureBox1.Width - pictureBox1.Width / 2 + tempConf.qP - 35, pictureBox1.Height - D3 + 2, pictureBox1.Width - pictureBox1.Width / 2 + tempConf.qP + 35, pictureBox1.Height - D3 + 2);// рисуем платформу
            g.FillEllipse(Brushes.Blue, pictureBox1.Width - pictureBox1.Width / 2 + tempConf.Xglob - D / 2, pictureBox1.Height - tempConf.Yglob - D / 2, D, D); //рисуем кружок  звена
            g.FillEllipse(Brushes.Green, pictureBox1.Width - pictureBox1.Width / 2 + tempConf.Xglob2 - D / 2, pictureBox1.Height - tempConf.Yglob2 - D / 2, D, D); //рисуем кружок  звена
            g.DrawLine(blackPen, pictureBox1.Width - pictureBox1.Width / 2 + tempConf.qP, pictureBox1.Height - 18, pictureBox1.Width - pictureBox1.Width / 2 + tempConf.Xglob, pictureBox1.Height - tempConf.Yglob);// "-15" это чтобы поднять основу манипулятора на 15 пикселей, по факту она от земли //по идее нужно переделать, чтобы манипулятор был на 15 пикселей выше
            g.DrawLine(blackPen, pictureBox1.Width - pictureBox1.Width / 2 + tempConf.Xglob, pictureBox1.Height - tempConf.Yglob, pictureBox1.Width - pictureBox1.Width / 2 + tempConf.Xglob2, pictureBox1.Height - tempConf.Yglob2);
            g.FillEllipse(Brushes.Blue, pictureBox1.Width - pictureBox1.Width / 2 + tempConf.Xglob3 - D / 2, pictureBox1.Height - tempConf.Yglob3 - D / 2, D, D); //рисуем кружок  звена
            g.FillEllipse(Brushes.Green, pictureBox1.Width - pictureBox1.Width / 2 + tempConf.Xglob4 - D / 2, pictureBox1.Height - tempConf.Yglob4 - D / 2, D, D); //рисуем кружок  звена
            g.DrawLine(blackPen, pictureBox1.Width - pictureBox1.Width / 2 + tempConf.Xglob2, pictureBox1.Height - tempConf.Yglob2, pictureBox1.Width - pictureBox1.Width / 2 + tempConf.Xglob3, pictureBox1.Height - tempConf.Yglob3);
            g.DrawLine(blackPen, pictureBox1.Width - pictureBox1.Width / 2 + tempConf.Xglob3, pictureBox1.Height - tempConf.Yglob3, pictureBox1.Width - pictureBox1.Width / 2 + tempConf.Xglob4, pictureBox1.Height - tempConf.Yglob4);
            paintObs();
            pictureBox1.Invalidate();
        }

        private void paintRobot()//отресовать все состояния робота
        {
            var D = 10;//радиус кружка звеньев
            Pen blackPen = new Pen(Color.Blue, 2);

            for (int i = 1; i < Rrt.backPathTT.Count; i++)
            {
                g.FillEllipse(Brushes.Blue, pictureBox1.Width - pictureBox1.Width / 2 + Rrt.backPathTT[i].V.Xglob - D / 2, pictureBox1.Height - Rrt.backPathTT[i].V.Yglob - D / 2, D, D); //рисуем кружок нулевого звена
                g.FillEllipse(Brushes.Green, pictureBox1.Width - pictureBox1.Width / 2 + Rrt.backPathTT[i].V.Xglob2 - D / 2, pictureBox1.Height - Rrt.backPathTT[i].V.Yglob2 - D / 2, D, D); //рисуем кружок  звена
                g.DrawLine(blackPen, pictureBox1.Width - pictureBox1.Width / 2, pictureBox1.Height, pictureBox1.Width - pictureBox1.Width / 2 + Rrt.backPathTT[i].V.Xglob, pictureBox1.Height - Rrt.backPathTT[i].V.Yglob);
                g.DrawLine(blackPen, pictureBox1.Width - pictureBox1.Width / 2 + Rrt.backPathTT[i].V.Xglob, pictureBox1.Height - Rrt.backPathTT[i].V.Yglob, pictureBox1.Width - pictureBox1.Width / 2 + Rrt.backPathTT[i].V.Xglob2, pictureBox1.Height - Rrt.backPathTT[i].V.Yglob2);
                System.Threading.Thread.Sleep(100);
                pictureBox1.Invalidate();
            }
        }

        private void button4_Click(object sender, EventArgs e)
        {
            Rrt.T.Clear();
            Rrt.backPath.Clear();
            Rrt.backPathTT.Clear();
            Rrt.success = false;
            i = 1;
            g.Clear(Color.White);
            paintObs();
            pictureBox1.Invalidate();
        }

        private void textBox3_TextChanged(object sender, EventArgs e)
        {

        }

        private void label2_Click(object sender, EventArgs e)
        {

        }

        private void pictureBox1_Click(object sender, EventArgs e)
        {
            if (e is MouseEventArgs)
            {
                var mea = e as MouseEventArgs;
                PointF coursor = new PointF(mea.X - pictureBox1.Width / 2, pictureBox1.Height - mea.Y);
                Obs.ObsList.Add(coursor);
                paintObs();
            }

        }

        private void paintObs()
        {
            int D = 30;
            for (int i = 0; i < Obs.ObsList.Count; i++)
            {
                g.FillEllipse(Brushes.Blue, pictureBox1.Width - pictureBox1.Width / 2 + Obs.ObsList[i].X - D / 2, pictureBox1.Height - Obs.ObsList[i].Y - D / 2, D, D); //рисуем кружок нулевого звена
            }
            pictureBox1.Invalidate();
        }

        private void Form1_Load(object sender, EventArgs e)
        {

        }

        private void button3_Click(object sender, EventArgs e)
        {
            Obs.ObsList.Clear();
            pictureBox1.Invalidate();
        }

        private void button5_Click(object sender, EventArgs e)
        {
            Rrt.T.Clear();
            Rrt.backPath.Clear();
            Rrt.backPathTT.Clear();
            Rrt.success = false;
            Obs.ObsList.Clear();
            i = 1;
            g.Clear(Color.White);
            pictureBox1.Invalidate();
        }
        private void label7_Click(object sender, EventArgs e)
        {

        }
    }
}
