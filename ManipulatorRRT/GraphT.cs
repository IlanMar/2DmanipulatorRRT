using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ManipulatorRRT
{
    public class Edge //EDGE// Ребро.
    {
        public PointF p1, p2;
        //public void SetEdge(PointF p1, PointF p2)
        //{
        //    this.p1 = p1;
        //    this.p2 = p2;
        //}
    }
    public class ManipulatorConf//параметры  манипулятора  //VERTEX
    {
        public float q, q2,q3,q4,qP; //огол относительно пред идущего звена// qP это координата платформы по оси Х
        public float linklenght, linklenght2, linklenght3, linklenght4;//длина манипулятора

        public float Xglob, Xglob2, Xglob3, Xglob4, XglobPlat;
        public float Yglob, Yglob2, Yglob3, Yglob4, YglobPlat;

        public float distanceToParent;
        public int parentId;


    }
    public class GraphT
    {

        public ManipulatorConf V = new ManipulatorConf(); //вершина графа, т.е конфигурация манипулятора
        public Edge E = new Edge();//ребро графа 
        //public GraphT(ManipulatorConf V, Edge E)
        //{
        //    this.V = V;
        //    this.E = E; 
        //}
    }

}
