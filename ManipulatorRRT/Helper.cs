using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ManipulatorRRT
{
   public  class Helper
    {
       public  ManipulatorConf GenerateTempState(float q, float q2)
        {
            //Random rand = new Random();
            ManipulatorConf newConf = new ManipulatorConf();
            newConf.q = q;//rand.Next(0, 180); //угл относительно предидущего звена
            newConf.q2 = q2;//rand.Next(0, 360); //угл относительно предидущего звена
            newConf.linklenght = 130;
            newConf.linklenght2 = 80; //длина второго звена

            //вычисляем координаты первого звена 
            var a = (newConf.linklenght) * Math.Cos((newConf.q) * (Math.PI / 180.0));// сначала градусы в радианы а потом синус из радианов в градусы
            var b = (newConf.linklenght) * Math.Sin((newConf.q) * (Math.PI / 180.0));
            newConf.Xglob = (float)a;
            newConf.Yglob = (float)b;
            // вычисляем координыта второго звена методом ПЗК аналитически
            var a2 = a + (newConf.linklenght2) * Math.Cos((newConf.q + newConf.q2) * (Math.PI / 180.0));// сначала градусы в радианы а потом синус из радианов в градусы
            var b2 = b + (newConf.linklenght2) * Math.Sin((newConf.q + newConf.q2) * (Math.PI / 180.0));
            newConf.Xglob2 = (float)a2;
            newConf.Yglob2 = (float)b2;

            return newConf;
        }

      
    }
}
