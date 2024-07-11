using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace PU4_MKP_Rational_Bezier
{
    // class for 3d point
    class Point3
    {
        public double x, y, z;
        public Point3(double _x, double _y, double _z)
        {
            x = _x;
            y = _y;
            z = _z;
        }
        public Point3(){}
    }
    // class for projective point
    class ProjectivePoint
    {
        public Point3 projected;
        public Point3 weighted;
        public double weight;

        public ProjectivePoint()
        {
            projected = new Point3();
            weighted = new Point3();
        }
        public ProjectivePoint(Point P, double w = 1)
        {
            weight = w;
            if(w != 0)
            {
                projected = new Point3(P.X, P.Y, 1);
                weighted = new Point3(w * P.X, w * P.Y, w);
            }
            else
            {
                projected = new Point3(P.X, P.Y, 1);
                weighted = new Point3(P.X, P.Y, 0);
            }
        }
        public ProjectivePoint(Point3 P)
        {
            weight = P.z;
            weighted = new Point3(P.x, P.y, P.z);
            if(weight != 0)
            {
                projected = new Point3(P.x / P.z, P.y / P.z, 1);
            }
            else
            {
                projected = new Point3(P.x, P.y, 1);
            }
        }
        public void SetPoint(double _x, double _y, double _z)
        {
            weight = _z;
            weighted = new Point3(_x, _y, _z);
            if (weight != 0)
            {
                projected = new Point3(_x / _z, _y / _z, 1);
            }
            else
            {
                projected = new Point3(_x, _y, 1);
            }
        }
        public void SetWeight(double w)
        {
            if(w != 0)
            {
                weighted = new Point3(projected.x * w, projected.y * w, w);
            }
            else
            {
                weighted = new Point3(projected.x, projected.y, 0);
            }
        }
    }
    public partial class MainWindow : Window
    {
        bool gen = false;
        List<ProjectivePoint> CV = new List<ProjectivePoint>();
        List<double> w = new List<double>();

        int? moving_point_index = null;
        double t;
        int noSegments = 40;

        List<Point> CurvePoints = new List<Point>();

        bool ell = false;
        Point? S = null;
        Point? P = null;

        bool inc = false;
        Point? T_A = null;
        Point? T_B = null;
        Point? T_C = null;

        bool vizualize = false;

        public MainWindow()
        {
            InitializeComponent();
        }
        private int CombNumber(int n, int k)
        {
            if (k == 0 || k == n) return 1;
            return CombNumber(n - 1, k - 1) + CombNumber(n - 1, k);
        }
        private double Power(double x, int n)
        {
            if (n == 0) return 1;
            if (n == 1) return x;

            if (n % 2 == 0) return Power(x, n / 2) * Power(x, n / 2);
            return Power(x, n / 2) * Power(x, n / 2) * x;
        }

        // Draw Casteljau algorithm
        private void DrawCasteljau(ref List<ProjectivePoint> ControlVertices)
        {
            List<List<ProjectivePoint>> points = new List<List<ProjectivePoint>>();
            points.Add(ControlVertices);
            for (int i = 1; i < ControlVertices.Count() - 1; ++i)
            {
                List<ProjectivePoint> tmp = new List<ProjectivePoint>();
                for (int j = 1; j < points[i - 1].Count(); ++j)
                {
                    double new_x = (1 - t) * points[i - 1][j - 1].weighted.x + t * points[i - 1][j].weighted.x;
                    double new_y = (1 - t) * points[i - 1][j - 1].weighted.y + t * points[i - 1][j].weighted.y;
                    double new_z = (1 - t) * points[i - 1][j - 1].weighted.z + t * points[i - 1][j].weighted.z;
                    ProjectivePoint P = new ProjectivePoint(new Point3(new_x, new_y, new_z));

                    tmp.Add(P);
                }
                points.Add(tmp);
            }

            for (int i = 1; i < points.Count(); ++i)
            {
                for (int j = 1; j < points[i].Count(); ++j)
                {
                    Line L = new Line();

                    L.Stroke = new SolidColorBrush(Colors.Black);
                    L.StrokeThickness = 0.5;

                    L.X1 = points[i][j - 1].projected.x;
                    L.Y1 = points[i][j - 1].projected.y;
                    L.X2 = points[i][j].projected.x;
                    L.Y2 = points[i][j].projected.y;

                    L.MouseLeftButtonDown += new MouseButtonEventHandler(g_MouseLeftButtonDown);
                    L.MouseLeftButtonUp += new MouseButtonEventHandler(g_MouseLeftButtonUp);

                    Canvas.SetZIndex(L, 5);
                    g.Children.Add(L);

                }
            }
            if (ControlVertices.Count() > 1)
            {
                ProjectivePoint Pt = new ProjectivePoint();
                double new_x = (1 - t) * points[points.Count() - 1][0].weighted.x + t * points[points.Count() - 1][1].weighted.x;
                double new_y = (1 - t) * points[points.Count() - 1][0].weighted.y + t * points[points.Count() - 1][1].weighted.y;
                double new_z = (1 - t) * points[points.Count() - 1][0].weighted.z + t * points[points.Count() - 1][1].weighted.z;

                Pt.SetPoint(new_x, new_y, new_z);

                Vector tangent_vector = new Vector(points[points.Count - 1][1].projected.x - points[points.Count - 1][0].projected.x,
                                                   points[points.Count - 1][1].projected.y - points[points.Count - 1][0].projected.y);

                Vector normal_vector = new Vector(-tangent_vector.Y, tangent_vector.X);

                Line tangent = new Line();

                tangent.X1 = Pt.projected.x;
                tangent.Y1 = Pt.projected.y;
                tangent.X2 = Pt.projected.x + tangent_vector.X;
                tangent.Y2 = Pt.projected.y + tangent_vector.Y;

                tangent.Stroke = Brushes.Blue;
                tangent.StrokeThickness = 2;

                Canvas.SetZIndex(tangent, 7);

                g.Children.Add(tangent);

                Line normal = new Line();

                normal.X1 = Pt.projected.x;
                normal.Y1 = Pt.projected.y;
                normal.X2 = Pt.projected.x + normal_vector.X;
                normal.Y2 = Pt.projected.y + normal_vector.Y;

                normal.Stroke = Brushes.Green;
                normal.StrokeThickness = 2;

                Canvas.SetZIndex(normal, 7);

                g.Children.Add(normal);

                Ellipse E = new Ellipse();
                E.Width = 10;
                E.Height = 10;
                E.Fill = new SolidColorBrush(Colors.OrangeRed);


                E.MouseLeftButtonDown += new MouseButtonEventHandler(g_MouseLeftButtonDown);
                E.MouseLeftButtonUp += new MouseButtonEventHandler(g_MouseLeftButtonUp);
                E.MouseRightButtonDown += new MouseButtonEventHandler(g_MouseRightButtonDown);
                E.MouseMove += new MouseEventHandler(g_MouseMove);

                Canvas.SetLeft(E, Pt.projected.x - 5);
                Canvas.SetTop(E, Pt.projected.y - 5);

                Canvas.SetZIndex(E, 7);

                g.Children.Add(E);
            }
        }
        // Draw Rational Bezier curve
        private void Draw_Bezier(ref List<ProjectivePoint> ControlVertices)
        {
            CurvePoints.Clear();
            double dt = 1.0 / noSegments;
            for(double _t = 0; _t < 1 + dt/2; _t+=dt)
            {
                List<List<ProjectivePoint>> points = new List<List<ProjectivePoint>>();
                points.Add(ControlVertices);
                for (int i = 1; i < ControlVertices.Count() - 1; ++i)
                {
                    List<ProjectivePoint> tmp = new List<ProjectivePoint>();
                    for (int j = 1; j < points[i - 1].Count(); ++j)
                    {
                        double new_x = (1 - _t) * points[i - 1][j - 1].weighted.x + _t * points[i - 1][j].weighted.x;
                        double new_y = (1 - _t) * points[i - 1][j - 1].weighted.y + _t * points[i - 1][j].weighted.y;
                        double new_z = (1 - _t) * points[i - 1][j - 1].weighted.z + _t * points[i - 1][j].weighted.z;
                        ProjectivePoint P = new ProjectivePoint(new Point3(new_x, new_y, new_z));

                        tmp.Add(P);
                    }
                    points.Add(tmp);
                }

                if (ControlVertices.Count() > 1)
                {
                    ProjectivePoint Pt = new ProjectivePoint();
                    double new_x = (1 - _t) * points[points.Count() - 1][0].weighted.x + _t * points[points.Count() - 1][1].weighted.x;
                    double new_y = (1 - _t) * points[points.Count() - 1][0].weighted.y + _t * points[points.Count() - 1][1].weighted.y;
                    double new_z = (1 - _t) * points[points.Count() - 1][0].weighted.z + _t * points[points.Count() - 1][1].weighted.z;
                    
                    Pt.SetPoint(new_x, new_y, new_z);

                    CurvePoints.Add(new Point(Pt.projected.x, Pt.projected.y));
                }
            }

            for(int i = 1; i<CurvePoints.Count; ++i)
            {
                Line L = new Line();
                L.X1 = CurvePoints[i - 1].X;
                L.Y1 = CurvePoints[i - 1].Y;
                L.X2 = CurvePoints[i].X;
                L.Y2 = CurvePoints[i].Y;

                L.Stroke = Brushes.Black;
                L.StrokeThickness = 2;

                g.Children.Add(L);
            }

        }
        private void incircle_Checked(object sender, RoutedEventArgs e)
        {
            gen = false;
            ell = false;
            inc = true;
            Reset();
        }

        private void ellipse_Checked(object sender, RoutedEventArgs e)
        {
            ell = true;
            gen = false;
            inc = false;
            Reset();
            
        }

        private void general_Checked(object sender, RoutedEventArgs e)
        {
            gen = true;
            ell = false;
            inc = false;
            Reset();
        }

        // Adding vertex
        private void g_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (gen)
            {
                Point click = e.GetPosition(g);
                ProjectivePoint proj_click = new ProjectivePoint(click, 1);
                int? close_index = IsClose(click);
                if (!close_index.HasValue)
                {
                    CV.Add(proj_click);
                    w.Add(1.0);
                    update_weights_TB();
                    Redraw();
                }
                else
                {
                    moving_point_index = close_index.Value;
                }

            }
            else if(ell)
            {
                if(!S.HasValue)
                {
                    S = e.GetPosition(g);
                    Redraw();
                }
                else if(!P.HasValue)
                {
                    P = e.GetPosition(g);
                    Redraw();
                }
                else
                {
                    S = e.GetPosition(g);
                    P = null;
                    Redraw();
                }
            }
            else if(inc)
            {
                if(!T_A.HasValue)
                {
                    T_A = e.GetPosition(g);
                    Redraw();
                }
                else if(!T_B.HasValue)
                {
                    T_B = e.GetPosition(g);
                    Redraw();
                }
                else if (!T_C.HasValue)
                {
                    T_C = e.GetPosition(g);
                    Redraw();
                }
                else
                {
                    T_A = e.GetPosition(g);
                    T_B = null;
                    T_C = null;
                    Redraw();
                }
            }
        }
        // Moving vertex
        private void g_MouseMove(object sender, MouseEventArgs e)
        {
            if (gen)
            {
                if (moving_point_index.HasValue)
                {
                    Point point_move = e.GetPosition(g);
                    ProjectivePoint proj_point_move = new ProjectivePoint(point_move, 1);
                    CV[moving_point_index.Value] = proj_point_move;
                    CV[moving_point_index.Value].SetWeight(w[moving_point_index.Value]);
                    Redraw();
                }
            }
        }
        // Stop moving the point
        private void g_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (gen)
            {
                if (moving_point_index.HasValue)
                {
                    moving_point_index = null;
                }
            }
        }
        private void Reset()
        {
            CV.Clear();
            w.Clear();
            CurvePoints.Clear();
            moving_point_index = null;
            if(weights != null)
                weights.Text = "";
            P = null;
            S = null;
            if(t_slider != null)
                t_slider.Value = 0.5;
            T_A = null;
            T_B = null;
            T_C = null;
            g.Children.Clear();
        }
        private void reset_Click(object sender, RoutedEventArgs e)
        {
            Reset();
        }
        private void update_weights_TB()
        {
            string w_text = "";
            for (int i = 0; i < w.Count; ++i)
            {
                w_text = w_text + w[i].ToString() + " ";
            }
            weights.Text = w_text;
        }
        private void SetWeightsCV()
        {
            for (int i = 0; i<CV.Count; ++i)
            {
                ProjectivePoint proj_point_move = new ProjectivePoint(new Point(CV[i].projected.x, CV[i].projected.y), 1);
                CV[i] = proj_point_move;
                CV[i].SetWeight(w[i]);
            }
        }
        private void update_Click(object sender, RoutedEventArgs e)
        {
            if(gen)
            {
                string w_text = weights.Text;
                w = w_text.Split(' ').Where(x => double.TryParse(x, out _)).Select(double.Parse).ToList();
                if (w.Count != CV.Count)
                {
                    w = Enumerable.Repeat(1.0, CV.Count).ToList();
                    update_weights_TB();
                }
                SetWeightsCV();
                Redraw();
            }
        }

        private void t_slider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {       
            t = t_slider.Value;
            Redraw();
        }
        // true if point P is close to some control point
        private int? IsClose(Point P)
        {
            for (int i = 0; i < CV.Count(); ++i)
            {
                if ((P.X - CV[i].projected.x) * (P.X - CV[i].projected.x) + (P.Y - CV[i].projected.y) * (P.Y - CV[i].projected.y) < 25)
                {
                    return i;
                }
            }
            return null;
        }
        // Control vertices
        private void DrawBezierPoints(ref List<ProjectivePoint> ControlVertices)
        {
            for (int i = 0; i < ControlVertices.Count(); ++i)
            {

                Ellipse new_ellipse = new Ellipse();
                new_ellipse.Width = 6;
                new_ellipse.Height = 6;
                new_ellipse.Fill = new SolidColorBrush(Colors.Orange);
                Canvas.SetLeft(new_ellipse, ControlVertices[i].projected.x - 3);
                Canvas.SetTop(new_ellipse, ControlVertices[i].projected.y - 3);
                Canvas.SetZIndex(new_ellipse, 4);

                new_ellipse.MouseLeftButtonUp += new MouseButtonEventHandler(g_MouseLeftButtonUp);
                new_ellipse.MouseLeftButtonDown += new MouseButtonEventHandler(g_MouseLeftButtonDown);

                g.Children.Add(new_ellipse);
            }
        }
        // Control polygon
        private void DrawBezierLines(ref List<ProjectivePoint> ControlVertices)
        {
            for (int i = 1; i < ControlVertices.Count(); ++i)
            {
                Line L = new Line();
                L.X1 = ControlVertices[i - 1].projected.x;
                L.Y1 = ControlVertices[i - 1].projected.y;
                L.X2 = ControlVertices[i].projected.x;
                L.Y2 = ControlVertices[i].projected.y;

                L.Stroke = new SolidColorBrush(Colors.Black);
                L.StrokeThickness = 1;

                L.MouseLeftButtonDown += new MouseButtonEventHandler(g_MouseLeftButtonDown);
                L.MouseLeftButtonUp += new MouseButtonEventHandler(g_MouseLeftButtonUp);

                Canvas.SetZIndex(L, 3);
                g.Children.Add(L);
            }
        }
        // Calculate points and weights for ellipse
        private void CalculateEllipse()
        {
            double a = Math.Abs(P.Value.X - S.Value.X);
            double b = Math.Abs(P.Value.Y - S.Value.Y);
            Point A = new Point(S.Value.X + a, S.Value.Y);
            Point B = new Point(S.Value.X, S.Value.Y + b); // w = 2
            Point C = new Point(S.Value.X - a, S.Value.Y);
            Point D = new Point(S.Value.X, S.Value.Y - b); // w = 2

            Point P_AB = new Point(S.Value.X + a, S.Value.Y + b);
            Point P_BC = new Point(S.Value.X - a, S.Value.Y + b);
            Point P_CD = new Point(S.Value.X - a, S.Value.Y - b);
            Point P_DA = new Point(S.Value.X + a, S.Value.Y - b);

            List<ProjectivePoint> CV1 = new List<ProjectivePoint>();

            CV1.Add(new ProjectivePoint(B, 2));
            CV1.Add(new ProjectivePoint(P_AB, 1));
            CV1.Add(new ProjectivePoint(A, 1));

            DrawBezierPoints(ref CV1);
            DrawBezierLines(ref CV1);
            Draw_Bezier(ref CV1);
            if (vizualize)
                DrawCasteljau(ref CV1);

            CV1.Clear();

            CV1.Add(new ProjectivePoint(C, 1));
            CV1.Add(new ProjectivePoint(P_BC, 1));
            CV1.Add(new ProjectivePoint(B, 2));

            DrawBezierPoints(ref CV1);
            DrawBezierLines(ref CV1);
            Draw_Bezier(ref CV1);
            if (vizualize)
                DrawCasteljau(ref CV1);

            CV1.Clear();

            CV1.Add(new ProjectivePoint(D, 2));
            CV1.Add(new ProjectivePoint(P_CD, 1));
            CV1.Add(new ProjectivePoint(C, 1));

            DrawBezierPoints(ref CV1);
            DrawBezierLines(ref CV1);
            Draw_Bezier(ref CV1);
            if (vizualize)
                DrawCasteljau(ref CV1);

            CV1.Clear();

            CV1.Add(new ProjectivePoint(A, 1));
            CV1.Add(new ProjectivePoint(P_DA, 1));
            CV1.Add(new ProjectivePoint(D, 2));

            DrawBezierPoints(ref CV1);
            DrawBezierLines(ref CV1);
            Draw_Bezier(ref CV1);
            if (vizualize)
                DrawCasteljau(ref CV1);

        }
        private void DrawPointsTriangle()
        {
            if (T_A.HasValue)
            {
                Ellipse E = new Ellipse();
                E.Width = 6;
                E.Height = 6;
                E.Fill = new SolidColorBrush(Colors.Black);

                E.MouseLeftButtonDown += new MouseButtonEventHandler(g_MouseLeftButtonDown);
                E.MouseLeftButtonUp += new MouseButtonEventHandler(g_MouseLeftButtonUp);
                E.MouseRightButtonDown += new MouseButtonEventHandler(g_MouseRightButtonDown);
                E.MouseMove += new MouseEventHandler(g_MouseMove);

                Canvas.SetLeft(E, T_A.Value.X - 3);
                Canvas.SetTop(E, T_A.Value.Y - 3);

                Canvas.SetZIndex(E, 7);

                g.Children.Add(E);
            }
            if (T_B.HasValue)
            {
                Ellipse E = new Ellipse();
                E.Width = 6;
                E.Height = 6;
                E.Fill = new SolidColorBrush(Colors.Black);

                E.MouseLeftButtonDown += new MouseButtonEventHandler(g_MouseLeftButtonDown);
                E.MouseLeftButtonUp += new MouseButtonEventHandler(g_MouseLeftButtonUp);
                E.MouseRightButtonDown += new MouseButtonEventHandler(g_MouseRightButtonDown);
                E.MouseMove += new MouseEventHandler(g_MouseMove);

                Canvas.SetLeft(E, T_B.Value.X - 3);
                Canvas.SetTop(E, T_B.Value.Y - 3);

                Canvas.SetZIndex(E, 7);

                g.Children.Add(E);

                Line L = new Line();

                L.X1 = T_A.Value.X;
                L.Y1 = T_A.Value.Y;
                L.X2 = T_B.Value.X;
                L.Y2 = T_B.Value.Y;

                L.Stroke = new SolidColorBrush(Colors.Black);
                L.StrokeThickness = 0.5;

                L.MouseLeftButtonDown += new MouseButtonEventHandler(g_MouseLeftButtonDown);
                L.MouseLeftButtonUp += new MouseButtonEventHandler(g_MouseLeftButtonUp);

                Canvas.SetZIndex(L, 3);
                g.Children.Add(L);
            }
            if (T_C.HasValue)
            {
                Ellipse E = new Ellipse();
                E.Width = 6;
                E.Height = 6;
                E.Fill = new SolidColorBrush(Colors.Black);

                E.MouseLeftButtonDown += new MouseButtonEventHandler(g_MouseLeftButtonDown);
                E.MouseLeftButtonUp += new MouseButtonEventHandler(g_MouseLeftButtonUp);
                E.MouseRightButtonDown += new MouseButtonEventHandler(g_MouseRightButtonDown);
                E.MouseMove += new MouseEventHandler(g_MouseMove);

                Canvas.SetLeft(E, T_C.Value.X - 3);
                Canvas.SetTop(E, T_C.Value.Y - 3);

                Canvas.SetZIndex(E, 7);

                g.Children.Add(E);

                Line L = new Line();

                L.X1 = T_B.Value.X;
                L.Y1 = T_B.Value.Y;
                L.X2 = T_C.Value.X;
                L.Y2 = T_C.Value.Y;

                L.Stroke = new SolidColorBrush(Colors.Black);
                L.StrokeThickness = 0.5;

                L.MouseLeftButtonDown += new MouseButtonEventHandler(g_MouseLeftButtonDown);
                L.MouseLeftButtonUp += new MouseButtonEventHandler(g_MouseLeftButtonUp);

                Canvas.SetZIndex(L, 3);
                g.Children.Add(L);


                Line L1 = new Line();

                L1.X1 = T_A.Value.X;
                L1.Y1 = T_A.Value.Y;
                L1.X2 = T_C.Value.X;
                L1.Y2 = T_C.Value.Y;

                L1.Stroke = new SolidColorBrush(Colors.Black);
                L1.StrokeThickness = 0.5;

                L1.MouseLeftButtonDown += new MouseButtonEventHandler(g_MouseLeftButtonDown);
                L1.MouseLeftButtonUp += new MouseButtonEventHandler(g_MouseLeftButtonUp);

                Canvas.SetZIndex(L1, 3);
                g.Children.Add(L1);
            }
        }
        private void DrawPointsEllipse()
        {
            if(S.HasValue)
            {
                Ellipse E = new Ellipse();
                E.Width = 6;
                E.Height = 6;
                E.Fill = new SolidColorBrush(Colors.Black);


                E.MouseLeftButtonDown += new MouseButtonEventHandler(g_MouseLeftButtonDown);
                E.MouseLeftButtonUp += new MouseButtonEventHandler(g_MouseLeftButtonUp);
                E.MouseRightButtonDown += new MouseButtonEventHandler(g_MouseRightButtonDown);
                E.MouseMove += new MouseEventHandler(g_MouseMove);

                Canvas.SetLeft(E, S.Value.X - 3);
                Canvas.SetTop(E, S.Value.Y - 3);

                Canvas.SetZIndex(E, 7);

                g.Children.Add(E);
            }
            if(P.HasValue)
            {
                Ellipse E = new Ellipse();
                E.Width = 6;
                E.Height = 6;
                E.Fill = new SolidColorBrush(Colors.Black);


                E.MouseLeftButtonDown += new MouseButtonEventHandler(g_MouseLeftButtonDown);
                E.MouseLeftButtonUp += new MouseButtonEventHandler(g_MouseLeftButtonUp);
                E.MouseRightButtonDown += new MouseButtonEventHandler(g_MouseRightButtonDown);
                E.MouseMove += new MouseEventHandler(g_MouseMove);

                Canvas.SetLeft(E, P.Value.X - 3);
                Canvas.SetTop(E, P.Value.Y - 3);

                Canvas.SetZIndex(E, 7);

                g.Children.Add(E);

                Line L = new Line();

                L.X1 = S.Value.X;
                L.Y1 = S.Value.Y;
                L.X2 = P.Value.X;
                L.Y2 = P.Value.Y;

                L.Stroke = new SolidColorBrush(Colors.Black);
                L.StrokeThickness = 0.5;

                L.MouseLeftButtonDown += new MouseButtonEventHandler(g_MouseLeftButtonDown);
                L.MouseLeftButtonUp += new MouseButtonEventHandler(g_MouseLeftButtonUp);

                Canvas.SetZIndex(L, 3);
                g.Children.Add(L);
            }
        }
        // Calculate points and weights for incircle
        private void CalculateIncircle()
        {
            double c = new Vector(T_B.Value.X - T_A.Value.X, T_B.Value.Y - T_A.Value.Y).Length;
            double a = new Vector(T_C.Value.X - T_B.Value.X, T_C.Value.Y - T_B.Value.Y).Length;
            double b = new Vector(T_C.Value.X - T_A.Value.X, T_C.Value.Y - T_A.Value.Y).Length;
            Point t_A = T_A.Value;
            Point t_B = T_B.Value;
            Point t_C = T_C.Value;

            // InCenter point: https://en.wikipedia.org/wiki/Incenter
            Point InCenter = new Point((a * t_A.X + b * t_B.X + c * t_C.X) / (a + b + c), (a * t_A.Y + b * t_B.Y + c * t_C.Y) / (a + b + c));

            // translate InCenter to the point (0, 0)
            Point A = new Point(t_A.X - InCenter.X, t_A.Y - InCenter.Y);
            Point B = new Point(t_B.X - InCenter.X, t_B.Y - InCenter.Y);
            Point C = new Point(t_C.X - InCenter.X, t_C.Y - InCenter.Y);

            // calculate touch points of incircle
            double s = (- A.X * (B.X - A.X) - A.Y * (B.Y - A.Y)) /
                ((B.X - A.X) * (B.X - A.X) + (B.Y - A.Y) * (B.Y - A.Y));

            // translated AB touch
            Point translated_P = new Point(A.X + s * (B.X - A.X), A.Y + s * (B.Y - A.Y));

            s = (-B.X * (C.X - B.X) - B.Y * (C.Y - B.Y)) /
                ((C.X - B.X) * (C.X - B.X) + (C.Y - B.Y) * (C.Y - B.Y));

            // translated BC touch
            Point translated_Q = new Point(B.X + s * (C.X - B.X), B.Y + s * (C.Y - B.Y));

            s = (-C.X * (A.X - C.X) - C.Y * (A.Y - C.Y)) /
                ((A.X - C.X) * (A.X - C.X) + (A.Y - C.Y) * (A.Y - C.Y));

            // translated CA touch
            Point translated_R = new Point(C.X + s * (A.X - C.X), C.Y + s * (A.Y - C.Y));

            // AB touch
            Point _P = new Point(InCenter.X + translated_P.X, InCenter.Y + translated_P.Y);
            // BC touch
            Point _Q = new Point(InCenter.X + translated_Q.X, InCenter.Y + translated_Q.Y);
            // CA touch
            Point _R = new Point(InCenter.X + translated_R.X, InCenter.Y + translated_R.Y); 

            // PQ.Length / 2
            double opp1 = new Vector(_P.X - _Q.X, _P.Y - _Q.Y).Length/2;
            // BQ.Length
            double hyp1 = new Vector(t_B.X - _Q.X, t_B.Y - _Q.Y).Length;

            // PR.Length / 2
            double opp2 = new Vector(_P.X - _R.X, _P.Y - _R.Y).Length / 2;
            // AR.Length
            double hyp2 = new Vector(t_A.X - _R.X, t_A.Y - _R.Y).Length;

            // RQ.Length / 2
            double opp3 = new Vector(_R.X - _Q.X, _R.Y - _Q.Y).Length / 2;
            // CQ.Length
            double hyp3 = new Vector(t_C.X - _Q.X, t_C.Y - _Q.Y).Length;

            // sine of half of the angle ABC
            double wB = opp1 / hyp1;
            // sine of half of the angle BAC
            double wA = opp2 / hyp2;
            // sine of half of the angle BCA
            double wC = opp3 / hyp3;
            
            ProjectivePoint Proj_B = new ProjectivePoint(t_B);
            Proj_B.SetWeight(wB);
            List<ProjectivePoint> CV1 = new List<ProjectivePoint>();
            CV1.Add(new ProjectivePoint(_Q));
            CV1.Add(Proj_B);
            CV1.Add(new ProjectivePoint(_P));

            DrawBezierPoints(ref CV1);
            DrawBezierLines(ref CV1);
            Draw_Bezier(ref CV1);
            if (vizualize)
                DrawCasteljau(ref CV1);
            
            ProjectivePoint Proj_C = new ProjectivePoint(t_C);
            Proj_C.SetWeight(wC);
            CV1.Clear();
            CV1.Add(new ProjectivePoint(_R));
            CV1.Add(Proj_C);
            CV1.Add(new ProjectivePoint(_Q));

            DrawBezierPoints(ref CV1);
            DrawBezierLines(ref CV1);
            Draw_Bezier(ref CV1);
            if (vizualize)
                DrawCasteljau(ref CV1);

            ProjectivePoint Proj_A = new ProjectivePoint(t_A);
            Proj_A.SetWeight(wA);
            CV1.Clear();
            CV1.Add(new ProjectivePoint(_P));
            CV1.Add(Proj_A);
            CV1.Add(new ProjectivePoint(_R));

            DrawBezierPoints(ref CV1);
            DrawBezierLines(ref CV1);
            Draw_Bezier(ref CV1);
            if (vizualize)
                DrawCasteljau(ref CV1);
        }

        // Redrawing function
        private void Redraw()
        {
            if(gen)
            {
                g.Children.Clear();
                DrawBezierPoints(ref CV);
                DrawBezierLines(ref CV);
                if (CV.Count > 1)
                {
                    if(vizualize)
                        DrawCasteljau(ref CV);
                    Draw_Bezier(ref CV);
                }
            }
            else if(ell)
            {
                g.Children.Clear();
                DrawPointsEllipse();
                if (P.HasValue)
                {
                    CalculateEllipse();
                }
            }
            else if(inc)
            {
                g.Children.Clear();
                DrawPointsTriangle();
                if(T_C.HasValue)
                {
                    CalculateIncircle();
                }
            }
        }
        private void g_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (gen)
            {
                Point click = e.GetPosition(g);
                int? close_index = IsClose(click);
                if (close_index.HasValue)
                {
                    CV.RemoveAt(close_index.Value);
                    w.RemoveAt(close_index.Value);
                    update_weights_TB();
                    SetWeightsCV();
                    Redraw();
                }
            }
        }

        // vizualize computations
        private void CheckBox_Checked(object sender, RoutedEventArgs e)
        {
            vizualize = true;
            Redraw();
        }

        // not vizualize computations
        private void CheckBox_Unchecked(object sender, RoutedEventArgs e)
        {
            vizualize = false;
            Redraw();
        }
    }
}
