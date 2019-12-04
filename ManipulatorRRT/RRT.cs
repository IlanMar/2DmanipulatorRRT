//Ilan Margolin 2019
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ManipulatorRRT
{    

    class RRT
    {
       // Form1 form = new Form1();
        public List<GraphT> T = new List<GraphT>(); // Список из Т где Т это вершина + ребро(V,E).
        ManipulatorConf V = new ManipulatorConf(); //вершина графа V, т.е конфигурация манипулятора
        Edge E = new Edge();// ребро графа       
        public bool success = false;
        public List<PointF> backPath = new List<PointF>();//список координат вершин обратного пути от цели к основанию
        public List<GraphT> backPathTT = new List<GraphT>(); // Список из Т где Т это вершина + ребро(V,E).
        Random rand = new Random();
        Random rand2 = new Random();
        Random rand3 = new Random();
        Random rand4 = new Random();
        Random vrand = new Random();
        public bool RRTStart(PointF Cinit, PointF Cgoal, int Nsteps, int Nextend, int edgeMaxLenght, List<PointF> obsList)// Edge P)// List<PointF> obsList
        {
            
            // 1. Шаг T(V.E)={Cinit, 0}
            GraphT GT = new GraphT();// объект вершина
            
          //  T.Add(GT);//список вершин
            // 2. Шаг step =0
            int step = 0;
            ManipulatorConf Ct = GenerateNewStare2(step);   // 728387
            GT.V = Ct;
            T.Add(GT);
            //3. Шаг success = false
           
            // 4. Шаг Начало цикла
            while ((step < Nsteps) && (success == false))
            {

              //  form.paintTree(T);
                // 5. шаг пропущен
                // 6. 
                ManipulatorConf Crand = GenerateNewStare(step+8876, Cgoal);
                bool g = intersectionsCheck(Crand, obsList);
                // 7 и 8 шаги пропущены
                // 9. Шаг
                float dist = distanceToNeighbor(Crand, T);

                if (8< dist && dist < edgeMaxLenght && g)// максимальная длина веток
                {
                    ManipulatorConf Cnear = NearestNeighbor(Crand, T);
                    ManipulatorConf Cnew = FindStoppingState(Cnear, Crand);//пока вернем Crand
                    //11. Шаг
                    if (Cnew != Cnear && Cnear.Xglob4 != Cnew.Xglob4&&Cnear.Yglob4 != Cnew.Yglob4)
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

                        success = (Distance(Eb, Cgoal) <= 15);
                    }
                }
                step = step + 1;
            }
            return success;
        }

        float Distance(Edge Eb, PointF Cgoal) 
        {
            var a = (Eb.p2.X - Cgoal.X);
            var b = (Eb.p2.Y - Cgoal.Y);            
            //var y = Math.Pow((a * a + b * b), 0.5);

            var y = ((a * a + b * b)* 0.5);
            return (float)y;
        }

        ManipulatorConf NearestNeighbor(ManipulatorConf Crand, List<GraphT>  T)
        {
                if (T.Count == 0) { return Crand; }
                double temp = 9999;
                int tempI = 0;
                for (int i = 0; i < T.Count - 1; i++)
                {
                    //var y = Math.Pow((Math.Pow(Mr.manipulatorLinks[i].Xglob4 - q1.Xglob4, stepen) + Math.Pow(Mr.manipulatorLinks[i].Yglob4 - q1.Yglob4, stepen)), 1 / 2);
                    var a = (T[i].V.Xglob4 - Crand.Xglob4);
                    var b = (T[i].V.Yglob4 - Crand.Yglob4);
                    var c = 0.5;
                    var y = Math.Pow((a * a + b * b), 0.5);
                    if (y < temp && genCoord(Crand, T[i].V)) 
                    {
                        tempI = i;
                        temp = y; 
                    }
                }
                if (temp > 100) //если новая точка ближе чем 40 к любой из точек то тру
                {
                   // return Crand;
                }
                return T[tempI].V;
              //  else { Mr.manipulatorLinks.Remove(Mr.manipulatorLinks.Last()); }
           
        }
        bool genCoord(ManipulatorConf Crand, ManipulatorConf Tiv)
        {
            int i =0;
           // double temp = 9999;
           // int tempI = 0;
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
            if (k < 10) key1 = true;
            if (k2 < 15) key2 = true;
            if (k3 < 20) key3 = true;
            if (k4 < 20) key4 = true;
            // key1 = true; key2 = true;
            if (key1 && key2 && key3 && key4)
            {
                //tempI = i;
                //temp = y;
                return true;
            }
            return false;
        }
        float distanceToNeighbor(ManipulatorConf Crand, List<GraphT> T)
        {
            //if (T.Count == 0) { return 10; }
            double temp = 9999;
            int tempI = 0;
            for (int i = 0; i <= T.Count - 1; i++)
            {
                //var y = Math.Pow((Math.Pow(Mr.manipulatorLinks[i].Xglob4 - q1.Xglob4, stepen) + Math.Pow(Mr.manipulatorLinks[i].Yglob4 - q1.Yglob4, stepen)), 1 / 2);
                var a = (T[i].V.Xglob4 - Crand.Xglob4);
                var b = (T[i].V.Yglob4 - Crand.Yglob4);
                var c = 0.5;
                var y = Math.Pow((a * a + b * b), 0.5);

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
               // key1 = true; key2 = true;
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
      //  Random rand4 = new Random();
        ManipulatorConf GenerateNewStare2(int step)
        {

            //  Random RandomNumber1 = new Random(DateTime.Now.Millisecond);

            Random trand = new Random(step);
            Random trand2 = new Random(step + 1);
            Random trand3 = new Random(step + 2);
             Random trand4 = new Random(step+3);
            ManipulatorConf Crand = new ManipulatorConf();
            bool generateKey = true;
            while (generateKey)
            {
                Crand.q = trand.Next(0, 180); //угл относительно предидущего звена
                Crand.q2 = trand.Next(0, 360); //угл относительно предидущего звена
                Crand.q3 = trand2.Next(0, 360); //угл относительно предидущего звена//rand2
                Crand.q4 = trand3.Next(0, 360); //угл относительно предидущего звена//rand3
                Crand.qP = 0;
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

        ////////////ищем три минимальных значения в списке

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
        ////////////

        ManipulatorConf GenerateNewStare(int step, PointF Cgoal)
        {
            //здесь мы выбираем случайную вершину
            bool key = true;
            int hh = 0;
            //while (key)
            //{
            //    Random vrand2 = new Random(step);
            //    hh = vrand2.Next(0, T.Count);
            //    GraphT randomV = T[hh];
            //    key = visitedVertexes.Contains(hh);
            //    if (key == false) break;
            //    visitedVertexes.Add(hh);
            //    if (T.Count < 3) { key = false; break; }

            //}
             List<int> randomVs = new List<int>();
             for (int i = 0; i < T.Count; i++) 
             {
                 //создаем список с случайными вершинами из списка
                 int tempStep = 0;
                 tempStep = tempStep + T.Count;
                // Random vrand = new Random(tempStep);
                 int hh2 = vrand.Next(0, T.Count-1);
                 randomVs.Add(hh2);
                 tempStep++;
             }
             float tempDistanse = 1000f;
            int nearestPoint = 0;
         //if (step % 2 ==0) 
         //       {
         //           for (int i = 0; i < T.Count; i++)
         //           {//выбираем самую близкую точку к целевой
         //               PointF j = new PointF(T[randomVs[i]].V.Xglob4, T[randomVs[i]].V.Yglob4);//j это координаты текущей точки из списка
         //               PointF goalPoint = Cgoal;

         //               var a = (j.X - goalPoint.X);
         //               var b = (j.Y - goalPoint.Y);
         //               //  var c = 0.5;
         //               var y = Math.Pow((a * a + b * b), 0.5);

         //               float distanse = (float)y;
         //               if (distanse <= tempDistanse)
         //               {
         //                   tempDistanse = distanse;
         //                   nearestPoint = i;
         //               }
         //               //    key = visitedVertexes.Contains(hh);

         //               //     if (key == false) { visitedVertexes.Add(hh); break; }
         //               //      if (visitedVertexes.Count > 500) visitedVertexes.Clear();
         //           }
         //       }


            if (step % 2 == 0)
            {
                float min = float.MaxValue; int Imin = 0;
                float[] mins = new float[3];
                //for (int i = 0; i < 3; i++)
                //{
                for (int k = 0; k < T.Count; k++)
                {//выбираем самую близкую точку к целевой
                 ////////////ищем три минимальных значения в списке

                    //int[] GetMin3(int[] nums)
                    //{
                    //    int min = int.MaxValue, Imin = 0;
                    //    int[] mins = new int[3];
                    //    for (int i = 0; i < 3; i++)
                    //    {
                    //        for (int j = 0; j < nums.Length; j++)
                    //            if (nums[j] < min)
                    //            {
                    //                min = nums[j];
                    //                Imin = j;
                    //            }
                    //        mins[i] = min;
                    //        nums[Imin] = int.MaxValue;
                    //        min = int.MaxValue;
                    //    }
                    //    return mins;
                    //}
                    ////////////

                    PointF j = new PointF(T[k].V.Xglob4, T[k].V.Yglob4);//j это координаты текущей точки из списка
                    PointF goalPoint = Cgoal;

                    var a = (j.X - goalPoint.X);
                    var b = (j.Y - goalPoint.Y);
                    //  var c = 0.5;
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
                    //if (mins[i] < min)
                    //{
                    //    mins[i] = min;
                    //    // nums[Imin] = float.MaxValue;
                    //    min = int.MaxValue;
                    //}


                }
                //}
                //     }
            }
            randomVs.Clear();
             int nPoint = nearestPoint;//hh
          //  Random RandomNumber1 = new Random(DateTime.Now.Millisecond);

         //   Random rand = new Random(step);
        ///    Random rand2 = new Random(step + 1);
         //   Random rand3 = new Random(step + 2);
          //  Random rand4 = new Random(step+3);
          //  Random rand5 = new Random(step + 5);
            ManipulatorConf Crand = new ManipulatorConf();
            bool generateKey = true;
            int Yleft = -2000;
            int Yright = 2000;
                while (generateKey)
                {
                    if (step % 2 == 0)
                    {

                        Crand.q = T[nPoint].V.q + 4 - (float)rand.NextDouble() * 8;  //(5-2*rand.NextDouble(0, 5)-5);// rand.Next(0, 180); //угл относительно предыдущего звена

                        Crand.q2 = T[nPoint].V.q2 - 4 + (float)rand.NextDouble() * 8;            //+ (5 - 2 * rand2.Next(0, 5) - 5);//rand.Next(0, 360); //угл относительно предидущего звена

                        Crand.q3 = T[nPoint].V.q3 + 4 - (float)rand.NextDouble() * 8;            // + (5 - 2 * rand3.Next(0, 5) - 5); //rand2.Next(0, 360); //угл относительно предидущего звена//rand2

                        Crand.q4 = T[nPoint].V.q4 - 4 + (float)rand.NextDouble() * 8;             // + (5 - 2 * rand4.Next(0, 5) - 5);// rand3.Next(0, 360); //угл относительно предидущего звена//rand3
                   Crand.qP =  T[nPoint].V.qP - 10 +(float)rand.NextDouble() * 20;


                    }
                if (step % 2 != 0)
                {

                    int rrr = rand.Next(0, T.Count - 1);
                    Crand.q = T[rrr].V.q + 4 - (float)rand.NextDouble() * 8;  //(5-2*rand.NextDouble(0, 5)-5);// rand.Next(0, 180); //угл относительно предидущего звена

                    Crand.q2 = T[rrr].V.q2 - 4 + (float)rand.NextDouble() * 8;            //+ (5 - 2 * rand2.Next(0, 5) - 5);//rand.Next(0, 360); //угл относительно предидущего звена

                    Crand.q3 = T[rrr].V.q3 - 4 + (float)rand.NextDouble() * 8;            // + (5 - 2 * rand3.Next(0, 5) - 5); //rand2.Next(0, 360); //угл относительно предидущего звена//rand2

                    Crand.q4 = T[rrr].V.q4 + 4 - (float)rand.NextDouble() * 8;             // + (5 - 2 * rand4.Next(0, 5) - 5);// rand3.Next(0, 360); //угл относительно предидущего звена//rand3
                   Crand.qP =  T[rrr].V.qP + 10 - (float)rand.NextDouble() * 20;


                }


                Crand.linklenght = 80; 
                Crand.linklenght2 = 100; //длина второго звена
                Crand.linklenght3 = 100;// rand.Next(50, 150);//Crand.linklenght3 = 100;  //вернуть закоменченную часть чтобы звено не удленялось
                Crand.linklenght4 = 100;// rand.Next(50, 150);//Crand.linklenght4 = 100;

                //вычисляем координаты первого звена 
                var a = ((Crand.linklenght) * Math.Cos((Crand.q) * (Math.PI / 180.0))) + Crand.qP;// сначала градусы в радианы а потом синус из радианов в градусы
                var b = (Crand.linklenght) * Math.Sin((Crand.q) * (Math.PI / 180.0));// +Crand.qP;
                Crand.Xglob = (float)a;// +Crand.qP;
                Crand.Yglob = (float)b;// +Crand.qP;
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
                if (T.Count <3) {generateKey = false; continue;}
                for (int i = T.Count-1; i > 0; i--)
                {
                    if (Math.Abs(T[i].V.Xglob4 - Crand.Xglob4) <5f && Math.Abs(T[i].V.Yglob4 - Crand.Yglob4) < 5f)
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
        ManipulatorConf FindStoppingState(ManipulatorConf Cnear, ManipulatorConf Crand)
        {
            //ManipulatorConf V = new ManipulatorConf();
            return Crand; //временная заглушка

        }

        public void FindbackPath()//получаем путь от цели к первой конфигурации манипулятора
        {
            int i = T.Count-1;
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
                //backPath.Add(p2);
                i = findNextVinT(p1); 
                if (i == 0)
                {
                    //backPath.Add(p1);
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
                if (Math.Abs(T[i].E.p2.X - p1.X) < eds2 && Math.Abs(T[i].E.p2.Y - p1.Y) < eds2)  //(T[i].E.p2 == p1)
                {
                    tempFind = p1;
                    return i;
                }
            }
            //if (p1.X == T[0].V.q2 && p1.Y == T[0].V.q) { return 0; }
            return 0;
        }
       public Point[] obs = new Point[12];

       bool intersectionsCheck(ManipulatorConf Crand, List<PointF> obsList)
       {
           bool vertex = false;
           bool edge = false;
           //obs[0].X = 50; obs[0].Y = 40;
           //obs[1].X = 70; obs[1].Y = 40;
           //obs[2].X = 50; obs[2].Y = 130;
           //obs[3].X = 70; obs[3].Y = 130;
           ////второй прямоугольник
           //obs[4].X = 70; obs[4].Y = 110;
           //obs[5].X = 130; obs[5].Y = 110;
           //obs[6].X = 70; obs[6].Y = 130;
           //obs[7].X = 130; obs[7].Y = 130;
           ////третий блок
           //obs[8].X = 0; obs[8].Y = 220;
           //obs[9].X = 30; obs[9].Y = 220;
           //obs[10].X = 0; obs[10].Y = 290;
           //obs[11].X = 30; obs[11].Y = 290;


           ////    50  >=    Xglob4   or Xglob4 >=60 and 40 >= Yglob4 or Yglob4 >= 90
           //if ((obs[0].X > Crand.Xglob4 || Crand.Xglob4 > obs[1].X) || (obs[0].Y > Crand.Yglob4 || Crand.Yglob4 > obs[2].Y))
           //{
           //    if ((obs[4].X > Crand.Xglob4 || Crand.Xglob4 > obs[5].X) || (obs[4].Y > Crand.Yglob4 || Crand.Yglob4 > obs[6].Y))
           //    {
           //        //теперь проверяем промежуточные звенеья на попадание на препятствие
           //        if ((obs[0].X > Crand.Xglob || Crand.Xglob > obs[1].X) || (obs[0].Y > Crand.Yglob || Crand.Yglob > obs[2].Y))
           //        {
           //            if ((obs[4].X > Crand.Xglob || Crand.Xglob > obs[5].X) || (obs[4].Y > Crand.Yglob || Crand.Yglob > obs[6].Y))
           //            {
           //                //третье препяствие
           //                if ((obs[8].X > Crand.Xglob4 || Crand.Xglob4 > obs[9].X) || (obs[8].Y > Crand.Yglob4 || Crand.Yglob4 > obs[10].Y))
           //                {
           //                    if ((obs[4].X > Crand.Xglob4 || Crand.Xglob4 > obs[5].X) || (obs[4].Y > Crand.Yglob4 || Crand.Yglob4 > obs[6].Y))
           //                    {
           //                        if ((obs[8].X > Crand.Xglob3 || Crand.Xglob3 > obs[9].X) || (obs[8].Y > Crand.Yglob3 || Crand.Yglob3 > obs[10].Y))
           //                        {
           //                            if ((obs[4].X > Crand.Xglob3 || Crand.Xglob3 > obs[5].X) || (obs[4].Y > Crand.Yglob3 || Crand.Yglob3 > obs[6].Y))
           //                            {

           //                            //    vertex = true;
           //                            }
           //                        }
           //                    }
           //                }
           //            }
           //        }
           //    }
           //}
           float x2,y2,Xa,Ya,Xb,Yb;
           x2 = 0; y2 = 0; Ya = 0;

           float x = 0;
           float y = 0;
            int distToObs = 50;

            for (int i = 0; i < 3; i++)//i < 3 потому что мы пока не учитываетм отрезое от основания одо 1го звена
            {
                if (i == 0) { x = Crand.Xglob; y = Crand.Yglob; x2 = Crand.Xglob2; y2 = Crand.Yglob2; }
                if (i == 1) { x = Crand.Xglob2; y = Crand.Yglob2; x2 = Crand.Xglob3; y2 = Crand.Yglob3; }
                if (i == 2) { x = Crand.Xglob3; y = Crand.Yglob3; x2 = Crand.Xglob4; y2 = Crand.Yglob4; }
                // if (i == 3) { x = Crand.Xglob4; y = Crand.Yglob4; }
               
               // var Rab = Math.Sqrt((Xb - x) * (Xb - x) + (Yb - y) * (Yb - y));//расстояние между началом звена и 
                for (int o=0;o<obsList.Count; o++)
                {

                    var c1 = Math.Sqrt((obsList[o].X - x) * (obsList[o].X - x) + (obsList[o].Y - y) * (obsList[o].Y - y));// высчитываем гипотенузу c1 - расстояние между нижним звеном и препятствием
                    var c2 = Math.Sqrt((obsList[o].X - x2) * (obsList[o].X - x2) + (obsList[o].Y - y2) * (obsList[o].Y - y2));//расстояние между верхним звеном и препятствием
                    //var alpha = 
                    if (c1 < distToObs || c2 < distToObs)
                    {
                        return false;
                    }

                } 

            }

               // for (int i = 0; i < 4; i++)//i < 3 потому что мы пока не учитываетм отрезое от основания одо 1го звена
               //{
               //    for (int j = 0; j < 30; j++)
               //    {
               //        if (i == 0) { x = Crand.Xglob; y = Crand.Yglob; Xb =Crand.Xglob2; Yb= Crand.Yglob2;}
               //        if (i == 1) { x = Crand.Xglob2; y = Crand.Yglob2;Xb =Crand.Xglob3; Yb= Crand.Yglob3;}
               //        if (i == 2) { x = Crand.Xglob3; y = Crand.Yglob3; Xb =Crand.Xglob4; Yb= Crand.Yglob4;}
               //        if (i == 3) { x = Crand.Xglob4; y = Crand.Yglob4; }

                       
               //        Xa=x;
               //        Ya = y;
               //        ///////
               //        double distBetweenManipAndObs =31; 
               //        if (i != 3)
               //        {
                          
               //            var Rab = Math.Sqrt((Xb - Xa) * (Xb - Xa) + (Yb - Ya) * (Yb - Ya));
               //            var Rac = Rab / 30;
               //            Rac = Rac * j;
               //            var z = Rac / Rab;
               //           // z = 1;
               //            var Xc = Xa + (Xb - Xa) * z;
               //            var Yc = Ya + (Yb - Ya) * z;
               //            //x = (float)Xc;
               //            //y = (float)Yc;

               //            for (int obl = 0; obl < obsList.Count; obl++)//сравниваем дистанцияю между каждой точкой звена и кружком препяттвия
               //            {
               //                 distBetweenManipAndObs= Math.Sqrt((obsList[obl].X - Xc) * (obsList[obl].X - Xc) + (obsList[obl].Y - Yc) * (obsList[obl].Y - Yc));
               //            }


               //        }

               //        if (distBetweenManipAndObs < 30)
               //        {
               //            return false;
               //        }
              

               //    }

               //}
           

               // х1, у1 и х2,у2 - координаты вершин первого отрезка;
               //х3, у3 и х4,у4 - координаты вершин второго отрезка;


              // if (vertex) return true;
              return true;           
       }


        public void setT() 
        {
            //PointF p1 = new PointF(6, 6);
            //PointF p2 = new PointF(6, 6);
            //V.q = 1;
            //V.q2 = 2;
            //E.p1 = p1;
            //E.p2 = p2;
           // GraphT GT=new GraphT(V, E);            
           // T.Add(GT);

        }
       
    }

}
