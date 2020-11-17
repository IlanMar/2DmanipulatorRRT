using System.Drawing;

namespace ManipulatorRRT
{
    public class Edge //EDGE
    {
        public PointF p1, p2;    
    }
    public class ManipulatorConf//параметры  манипулятора  //VERTEX
    {
        public float q, q2,q3,q4,qP; //огол относительно пред идущего звена// qP это координата платформы по оси Х
        public float linklenght, linklenght2, linklenght3, linklenght4;//длина манипулятора
        public float Xglob, Xglob2, Xglob3, Xglob4, XglobPlat;
        public float Yglob, Yglob2, Yglob3, Yglob4, YglobPlat;
        public float distanceToParent;
        public int parentID;
    }

    public class GraphT
    {
        public ManipulatorConf V = new ManipulatorConf(); //вершина графа, т.е конфигурация манипулятора
        public Edge E = new Edge();//ребро графа     
    }
}
