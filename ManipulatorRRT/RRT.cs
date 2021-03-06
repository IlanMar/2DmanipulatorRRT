﻿//Ilan Margolin 2019
using System;
using System.Collections.Generic;
using System.Drawing;

namespace ManipulatorRRT
{
    class RRT
    {
        public List<GraphT> T = new List<GraphT>(); // Список из Т где Т это вершина + ребро(V,E).
        ManipulatorConf V = new ManipulatorConf(); //вершина графа V, т.е конфигурация манипулятора
        Edge E = new Edge(); // ребро графа       
        public bool success = false;
        public List<PointF> backPath = new List<PointF>();  //список координат вершин обратного пути от цели к основанию
        public List<GraphT> backPathTT = new List<GraphT>(); // Список из Т где Т это вершина + ребро(V,E).
        Random rand = new Random();
        Random rand2 = new Random();
        Random rand3 = new Random();
        Random rand4 = new Random();
        Random vrand = new Random();

        public bool RRTStart(PointF Cinit, PointF Cgoal, int Nsteps, int Nextend, int edgeMaxLenght, List<PointF> obsList, float qincrement, bool checkBox1, int dopustimueOtklonenia)// Edge P)// List<PointF> obsList
        {
            // 1. Шаг T(V.E)={Cinit, 0}
            GraphT GT = new GraphT();// объект вершина   
            // 2. Шаг 2
            int step = 0;
            ManipulatorConf Ct = GenerateNewStare2(72087);   //генерируем начальное положение манипулятора (пока что заданно жестко)
            GT.V = Ct;
            T.Add(GT);//список вершин
            //3. Шаг success = false
            // 4. Шаг Начало цикла
            while ((step < Nsteps) && (success == false))
            {
                // 5. шаг пропущен
                // 6.  
                ManipulatorConf Crand = GenerateNewStare(step + 8876, Cgoal, qincrement);  //GenerateNewState
                bool g = intersectionsCheck(Crand, obsList);  //передалать для 3Д
                // 7 и 8 шаги пропущены
                // 9. Шаг
                float dist = distanceToNeighbor(Crand, T);

                if (8 < dist && dist < edgeMaxLenght && g && Crand.distanceToParent < edgeMaxLenght)// максимальная длина веток
                {
                    ManipulatorConf Cnear = T[Crand.parentID].V;//все это для того чтобы через чек бокс можно было выбирать два варианта приращения

                    if (checkBox1)
                    {
                        Cnear = T[Crand.parentID].V;// NearestNeighbor(Crand, T, Crand.parentID);
                    }
                    else { Cnear = NearestNeighbor(Crand, T, Crand.parentID, dopustimueOtklonenia); }

                    ManipulatorConf Cnew = FindStoppingState(Cnear, Crand);//пока вернем Crand
                    float y = findDistanseBetweenV(Cnear, Cnew);

                    //11. Шаг
                    if (Cnew != Cnear && Cnear.Xglob4 != Cnew.Xglob4 && Cnear.Yglob4 != Cnew.Yglob4 & y < edgeMaxLenght)
                    {
                        Edge Eb = new Edge();
                        ManipulatorConf Vb = new ManipulatorConf();
                        Eb.p1.X = Cnear.Xglob4;//точка p1 это старый узел а p2 это новый узел
                        Eb.p1.Y = Cnear.Yglob4;
                        Eb.p2.X = Cnew.Xglob4;
                        Eb.p2.Y = Cnew.Yglob4;
                        var GT2 = new GraphT();
                        GT2 = new GraphT();
                        GT2.V = Cnew;
                        GT2.E = Eb;
                        T.Add(GT2);
                        success = (Distance(Eb, Cgoal) <= 20);
                    }
                }
                step = step + 1;
            }
            return success;
        }

        float Distance(Edge Eb, PointF Cgoal) // проверка на то, что новая вершина находится не дальше заданного значения от исходной
        {
            var a = (Eb.p2.X - Cgoal.X);
            var b = (Eb.p2.Y - Cgoal.Y);
            var y = ((a * a + b * b) * 0.5);
            return (float)y;
        }

        ManipulatorConf NearestNeighbor(ManipulatorConf Crand, List<GraphT> T, int parentID, int dopustimueOtklonenia)// 
        {
            if (T.Count == 0) { return Crand; }
            float temp = 99999;
            int tempI = 0;

            for (int i = 0; i < T.Count - 1; i++)
            {
                var a = (T[i].V.Xglob4 - Crand.Xglob4);
                var b = (T[i].V.Yglob4 - Crand.Yglob4);
                float c = a * a + b * b;
                float answer = (float)Math.Sqrt(c);

                if (answer < temp && genCoord(Crand, T[i].V, dopustimueOtklonenia))
                {
                    tempI = i;
                    temp = answer;
                }
            }

            if (temp > 100) //если новая точка ближе чем 40 к любой из точек то тру
            {
                // return Crand;
            }
            return T[tempI].V;
        }

        bool genCoord(ManipulatorConf Crand, ManipulatorConf Tiv, int dopustimueOtklonenia)
        {
            var a = (Tiv.Xglob4 - Crand.Xglob4);
            var b = (Tiv.Yglob4 - Crand.Yglob4);
            var c = 0.5;
            var y = Math.Pow((a * a + b * b), 0.5);
            //начинаем проверку чтобы угловые координаты не были слишком различны от точки к точке
            float k = Math.Abs(Tiv.q - Crand.q);
            float k2 = Math.Abs(Tiv.q2 - Crand.q2);
            float k3 = Math.Abs(Tiv.q3 - Crand.q3);
            float k4 = Math.Abs(Tiv.q4 - Crand.q4);

            if (k > 180)
            {
                k = 360 - k;
                k = k + Tiv.q;
            }
            if (k2 > 180)
            {
                k2 = 360 - k2;
                k2 = k2 + Tiv.q2;
            }
            if (k3 > 180)
            {
                k3 = 360 - k3;
                k3 = k3 + Tiv.q3;
            }
            if (k4 > 180)
            {
                k4 = 360 - k4;
                k4 = k4 + Tiv.q4;
            }

            bool key1 = false; bool key2 = false; bool key3 = false; bool key4 = false;

            if (k < dopustimueOtklonenia) key1 = true;//здесь задаются допустимые отклонения манипулятора при генерации новой точки
            if (k2 < dopustimueOtklonenia) key2 = true;
            if (k3 < dopustimueOtklonenia) key3 = true;
            if (k4 < dopustimueOtklonenia) key4 = true;
            if (key1 && key2 && key3 && key4)
                return true;

            return false;
        }

        float distanceToNeighbor(ManipulatorConf Crand, List<GraphT> T)
        {
            double temp = 9999;
            int tempI = 0;

            for (int i = 0; i <= T.Count - 1; i++)
            {
                var a = (T[i].V.Xglob4 - Crand.Xglob4);
                var b = (T[i].V.Yglob4 - Crand.Yglob4);
                float c = a * a + b * b;
                float y = (float)Math.Sqrt(c);
                //начинаем проверку чтобы угловые координаты не были слишком различны от точки к точке
                float k = Math.Abs(T[i].V.q - Crand.q);
                float k2 = Math.Abs(T[i].V.q2 - Crand.q2);
                float k3 = Math.Abs(T[i].V.q3 - Crand.q3);
                float k4 = Math.Abs(T[i].V.q4 - Crand.q4);

                if (k > 180)
                {
                    k = 360 - k;
                    k = k + T[i].V.q;
                }

                if (k2 > 180)
                {
                    k2 = 360 - k2;
                    k2 = k2 + T[i].V.q2;
                }

                if (k3 > 180)
                {
                    k3 = 360 - k3;
                    k3 = k3 + T[i].V.q3;
                }

                if (k4 > 180)
                {
                    k4 = 360 - k4;
                    k4 = k4 + T[i].V.q4;
                }

                bool key1 = false; bool key2 = false; bool key3 = false; bool key4 = false;
                if (k < 15) key1 = true;
                if (k2 < 15) key2 = true;
                if (k3 < 15) key3 = true;
                if (k4 < 15) key4 = true;
                if (y < temp && key1 && key2 && key3 && key4)
                {
                    tempI = i;
                    temp = y;
                }
            }

            if (temp > 50) //если новая точка ближе чем 40 к любой из точек то тру
            {
                // return Crand;
            }

            return (float)temp;
        }

        ManipulatorConf GenerateNewStare2(int step)
        {
            Random trand = new Random(step);
            Random trand2 = new Random(step + 1);
            Random trand3 = new Random(step + 2);
            Random trand4 = new Random(step + 3);
            ManipulatorConf Crand = new ManipulatorConf();
            bool generateKey = true;

            while (generateKey)
            {
                Crand.q = 85;//trand.Next(0, 180); //угл относительно предидущего звена
                Crand.q2 = 20;// trand.Next(0, 360); //угл относительно предидущего звена
                Crand.q3 = -40;// trand2.Next(0, 360); //угл относительно предидущего звена//rand2
                Crand.q4 = 40;//trand3.Next(0, 360); //угл относительно предидущего звена//rand3
                Crand.qP = -80;
                Crand.linklenght = 80;
                Crand.linklenght2 = 100; //длина второго звена
                Crand.linklenght3 = 100;
                Crand.linklenght4 = 100;
                //вычисляем координаты первого звена 
                var a = (Crand.linklenght) * Math.Cos((Crand.q) * (Math.PI / 180.0)) + Crand.qP;// сначала градусы в радианы а потом синус из радианов в градусы
                var b = (Crand.linklenght) * Math.Sin((Crand.q) * (Math.PI / 180.0));
                Crand.Xglob = (float)a;
                Crand.Yglob = (float)b;
                // вычисляем координыта второго звена методом ПЗК аналетически
                var a2 = a + (Crand.linklenght2) * Math.Cos((Crand.q + Crand.q2) * (Math.PI / 180.0));// сначала градусы в радианы а потом синус из радианов в градусы
                var b2 = b + (Crand.linklenght2) * Math.Sin((Crand.q + Crand.q2) * (Math.PI / 180.0));
                Crand.Xglob2 = (float)a2;
                Crand.Yglob2 = (float)b2;
                // вычисляем координыта второго звена методом ПЗК аналетически
                var a3 = a2 + (Crand.linklenght3) * Math.Cos((Crand.q + Crand.q2 + Crand.q3) * (Math.PI / 180.0));// сначала градусы в радианы а потом синус из радианов в градусы
                var b3 = b2 + (Crand.linklenght3) * Math.Sin((Crand.q + Crand.q2 + Crand.q3) * (Math.PI / 180.0));
                Crand.Xglob3 = (float)a3;
                Crand.Yglob3 = (float)b3;
                // вычисляем координыта второго звена методом ПЗК аналетически
                var a4 = a3 + (Crand.linklenght4) * Math.Cos((Crand.q + Crand.q2 + Crand.q3 + Crand.q4) * (Math.PI / 180.0));// сначала градусы в радианы а потом синус из радианов в градусы
                var b4 = b3 + (Crand.linklenght4) * Math.Sin((Crand.q + Crand.q2 + Crand.q3 + Crand.q4) * (Math.PI / 180.0));
                Crand.Xglob4 = (float)a4;
                Crand.Yglob4 = (float)b4;
                if (T.Count < 3) { generateKey = false; continue; }

                for (int i = T.Count - 1; i > 0; i--)
                {
                    if (Math.Abs(T[i].V.Xglob4 - Crand.Xglob4) < 5f && Math.Abs(T[i].V.Yglob4 - Crand.Yglob4) < 5f)
                    {
                        step = step + 1;
                        generateKey = true;
                        continue;
                    }
                    generateKey = false;
                }
            }
            return Crand;
        }

        List<int> visitedVertexes = new List<int>();//чтобы мы не создавали точки рядом с уже посещенными вершинами    

        int[] GetMin3(int[] nums)
        {
            int min = int.MaxValue, Imin = 0;
            int[] mins = new int[3];

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < nums.Length; j++)

                    if (nums[j] < min)
                    {
                        min = nums[j];
                        Imin = j;
                    }

                mins[i] = min;
                nums[Imin] = int.MaxValue;
                min = int.MaxValue;
            }
            return mins;
        }

        ManipulatorConf GenerateNewStare(int step, PointF Cgoal, float qincrement)//здесь мы выбираем случайную вершину
        {
            List<int> randomVs = new List<int>();

            for (int i = 0; i < T.Count; i++)
            {
                //создаем список с случайными вершинами из списка
                int tempStep = 0;
                tempStep = tempStep + T.Count;
                // Random vrand = new Random(tempStep);
                int hh2 = vrand.Next(0, T.Count - 1);
                randomVs.Add(hh2);
                tempStep++;
            }

            float tempDistanse = 1000f;
            int nearestPoint = 0;

            if (step % 2 == 0)
            {
                float min = float.MaxValue; int Imin = 0;
                float[] mins = new float[3];

                for (int k = 0; k < T.Count; k++)//выбираем самую близкую точку к целевой
                {
                    PointF j = new PointF(T[k].V.Xglob4, T[k].V.Yglob4);//j это координаты текущей точки из списка
                    PointF goalPoint = Cgoal;
                    var a = (j.X - goalPoint.X);
                    var b = (j.Y - goalPoint.Y);
                    var y = Math.Pow((a * a + b * b), 0.5);
                    float distanse = (float)y;

                    if (distanse <= tempDistanse)
                    {
                        tempDistanse = distanse;
                        nearestPoint = k;
                    }

                    if (y < min)
                    {
                        min = (float)y;
                        Imin = k;
                    }
                }
            }

            randomVs.Clear();
            int nPoint = nearestPoint;
            ManipulatorConf Crand = new ManipulatorConf();
            bool generateKey = true;         
            int rrr = rand.Next(0, T.Count - 1);

            while (generateKey)
            {
                if (step % 2 == 0)
                {
                    Crand.q = T[nPoint].V.q + qincrement - (float)rand.NextDouble() * (qincrement * 2);  //(5-2*rand.NextDouble(0, 5)-5);// rand.Next(0, 180); //угл относительно предыдущего звена
                    Crand.q2 = T[nPoint].V.q2 - qincrement + (float)rand.NextDouble() * (qincrement * 2);            //+ (5 - 2 * rand2.Next(0, 5) - 5);//rand.Next(0, 360); //угл относительно предидущего звена
                    Crand.q3 = T[nPoint].V.q3 + qincrement - (float)rand.NextDouble() * (qincrement * 2);            // + (5 - 2 * rand3.Next(0, 5) - 5); //rand2.Next(0, 360); //угл относительно предидущего звена//rand2
                    Crand.q4 = T[nPoint].V.q4 - qincrement + (float)rand.NextDouble() * (qincrement * 2);             // + (5 - 2 * rand4.Next(0, 5) - 5);// rand3.Next(0, 360); //угл относительно предидущего звена//rand3
                    Crand.qP = T[nPoint].V.qP - 10 + (float)rand.NextDouble() * 20;//закоментировать для зафиксирования основания
                }

                if (step % 2 != 0)
                {
                    Crand.q = T[rrr].V.q + qincrement - (float)rand.NextDouble() * (qincrement * 2);  //(5-2*rand.NextDouble(0, 5)-5);// rand.Next(0, 180); //угл относительно предидущего звена
                    Crand.q2 = T[rrr].V.q2 - qincrement + (float)rand.NextDouble() * (qincrement * 2);            //+ (5 - 2 * rand2.Next(0, 5) - 5);//rand.Next(0, 360); //угл относительно предидущего звена
                    Crand.q3 = T[rrr].V.q3 - qincrement + (float)rand.NextDouble() * (qincrement * 2);            // + (5 - 2 * rand3.Next(0, 5) - 5); //rand2.Next(0, 360); //угл относительно предидущего звена//rand2
                    Crand.q4 = T[rrr].V.q4 + qincrement - (float)rand.NextDouble() * (qincrement * 2);             // + (5 - 2 * rand4.Next(0, 5) - 5);// rand3.Next(0, 360); //угл относительно предидущего звена//rand3
                    Crand.qP = T[rrr].V.qP + 10 - (float)rand.NextDouble() * 20;   //закоментировать для зафиксирования основания
                }

                Crand.linklenght = 80;
                Crand.linklenght2 = 100; //длина второго звена
                Crand.linklenght3 = 100;// rand.Next(50, 150);  //вернуть закоменченную часть чтобы звено не удленялось
                Crand.linklenght4 = 100;// rand.Next(50, 150);
                //вычисляем координаты первого звена 
                var a = ((Crand.linklenght) * Math.Cos((Crand.q) * (Math.PI / 180.0))) + Crand.qP;// сначала градусы в радианы а потом синус из радианов в градусы
                var b = (Crand.linklenght) * Math.Sin((Crand.q) * (Math.PI / 180.0));// +Crand.qP;
                Crand.Xglob = (float)a;
                Crand.Yglob = (float)b;
                // вычисляем координыта второго звена методом ПЗК аналетически
                var a2 = a + (Crand.linklenght2) * Math.Cos((Crand.q + Crand.q2) * (Math.PI / 180.0));// сначала градусы в радианы а потом синус из радианов в градусы
                var b2 = b + (Crand.linklenght2) * Math.Sin((Crand.q + Crand.q2) * (Math.PI / 180.0));
                Crand.Xglob2 = (float)a2;
                Crand.Yglob2 = (float)b2;
                // вычисляем координыта третьего звена методом ПЗК аналетически
                var a3 = a2 + (Crand.linklenght3) * Math.Cos((Crand.q + Crand.q2 + Crand.q3) * (Math.PI / 180.0));// сначала градусы в радианы а потом синус из радианов в градусы
                var b3 = b2 + (Crand.linklenght3) * Math.Sin((Crand.q + Crand.q2 + Crand.q3) * (Math.PI / 180.0));
                Crand.Xglob3 = (float)a3;
                Crand.Yglob3 = (float)b3;
                // вычисляем координыта четвертого звена методом ПЗК аналетически
                var a4 = a3 + (Crand.linklenght4) * Math.Cos((Crand.q + Crand.q2 + Crand.q3 + Crand.q4) * (Math.PI / 180.0));// сначала градусы в радианы а потом синус из радианов в градусы
                var b4 = b3 + (Crand.linklenght4) * Math.Sin((Crand.q + Crand.q2 + Crand.q3 + Crand.q4) * (Math.PI / 180.0));
                Crand.Xglob4 = (float)a4;
                Crand.Yglob4 = (float)b4;

                if (T.Count < 3) { generateKey = false; continue; }

                for (int i = T.Count - 1; i > 0; i--)
                {
                    if (Math.Abs(T[i].V.Xglob4 - Crand.Xglob4) < 5f && Math.Abs(T[i].V.Yglob4 - Crand.Yglob4) < 5f)
                    {
                        step = step + 1;
                        generateKey = true;
                        continue;
                    }

                    generateKey = false;
                }
            }

            float a1 = 0, b1 = 0;

            if (step % 2 == 0)//вычисляем дистанцию до родительского звена
            {
                a1 = (T[rrr].V.Xglob4 - Crand.Xglob4);
                b1 = (T[rrr].V.Yglob4 - Crand.Yglob4);
                Crand.parentID = rrr;
            }

            if (step % 2 == 0)
            {
                a1 = (T[nPoint].V.Xglob4 - Crand.Xglob4);
                b1 = (T[nPoint].V.Yglob4 - Crand.Yglob4);
                Crand.parentID = nPoint;
            }

            float c = a1 * a1 + b1 * b1;  //теорема пифагора
            float y1 = (float)Math.Sqrt(c);//вычисляем дистанцию до родительского звена
            Crand.distanceToParent = y1;
            return Crand;
        }

        ManipulatorConf FindStoppingState(ManipulatorConf Cnear, ManipulatorConf Crand)
        {
            return Crand; //временная заглушка
        }

        public void FindbackPath()//получаем путь от цели к первой конфигурации манипулятора
        {
            int i = T.Count - 1;
            GraphT Vertex = T[i];
            PointF p1 = Vertex.E.p1;//корневая для нее вершина
            PointF p2 = Vertex.E.p2;//текущая вершина
            backPath.Add(p1);
            backPathTT.Add(T[i]);
            bool key = true;

            while (key)
            {

                GraphT Vertex2 = T[i];
                p1 = Vertex2.E.p1;//текущая вершина//Cnear старый узел
                p2 = Vertex2.E.p2;//корневая для нее вершина//Cnew  узел новее

                if (p1.X == 0)
                {

                }

                backPath.Add(p1);
                backPathTT.Add(T[i]);
                i = findNextVinT(p1);

                if (i == 0)
                {
                    backPathTT.Add(T[i]);
                    key = false;
                }
            }
        }

        PointF tempFind;

        int findNextVinT(PointF p1)
        {
            for (int i = T.Count - 1; i > -1; i--)
            {
                float eds = 0.01f;
                float eds2 = 0.02f;

                if (Math.Abs(T[i].E.p2.X - tempFind.X) < eds && Math.Abs(T[i].E.p2.Y - tempFind.Y) < eds)
                {
                    continue;
                }

                if (Math.Abs(T[i].E.p2.X - p1.X) < eds2 && Math.Abs(T[i].E.p2.Y - p1.Y) < eds2)
                {
                    tempFind = p1;
                    return i;
                }
            }
            return 0;
        }

        public Point[] obs = new Point[12];

        float findDistanseBetweenV(ManipulatorConf A, ManipulatorConf B)
        {
            var a = (A.Xglob4 - B.Xglob4);
            var b = (A.Yglob4 - B.Yglob4);
            float c = a * a + b * b;  //теорема пифагора =)
            float y = (float)Math.Sqrt(c);
            return y;
        }

        bool intersectionsCheck(ManipulatorConf Crand, List<PointF> obsList)
        {
            float x2, y2;
            x2 = 0; y2 = 0;
            float x = 0;
            float y = 0;
            int distToObs = 50;

            for (int i = 0; i < 3; i++)//i < 3 потому что мы пока не учитываетм отрезое от основания одо 1го звена
            {
                if (i == 0) { x = Crand.Xglob; y = Crand.Yglob; x2 = Crand.Xglob2; y2 = Crand.Yglob2; }
                if (i == 1) { x = Crand.Xglob2; y = Crand.Yglob2; x2 = Crand.Xglob3; y2 = Crand.Yglob3; }
                if (i == 2) { x = Crand.Xglob3; y = Crand.Yglob3; x2 = Crand.Xglob4; y2 = Crand.Yglob4; }

                for (int o = 0; o < obsList.Count; o++)
                {
                    var c1 = Math.Sqrt((obsList[o].X - x) * (obsList[o].X - x) + (obsList[o].Y - y) * (obsList[o].Y - y));// высчитываем гипотенузу c1 - расстояние между нижним звеном и препятствием
                    var c2 = Math.Sqrt((obsList[o].X - x2) * (obsList[o].X - x2) + (obsList[o].Y - y2) * (obsList[o].Y - y2));//расстояние между верхним звеном и препятствием

                    if (c1 < distToObs || c2 < distToObs)
                    {
                        return false;
                    }
                }
            }
            return true;
        }
    }
}
