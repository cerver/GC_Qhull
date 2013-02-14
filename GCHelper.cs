using System;
using System.Text;
using System.Text.RegularExpressions;
using System.IO;
using System.Diagnostics;
using System.Collections.Generic;
using System.ComponentModel;
using System.Threading;
using System.Reflection;
using Bentley.Geometry;
using Bentley.GenerativeComponents;
using Bentley.GenerativeComponents.Features.Specific;
using Bentley.GenerativeComponents.MicroStation;
using Bentley.Interop.MicroStationDGN;
using Bentley.GenerativeComponents.GCScript;
using Bentley.GenerativeComponents.GCScript.UISupport;
using Bentley.GenerativeComponents.XceedHelpers;
using Bentley.GenerativeComponents.Features;

namespace QhullHelper
{
    public class QhullUtility
    {
        public static void OffParser(string[] inString, int dim, bool filterInf,bool isD3d, out DPoint3d[][] outPoints)
        {
            //parse the output
           

            //get counts
            string[] ctLine = inString[1].Split(' ');
            int ptNum = int.Parse(ctLine[0]);
            int cellNum = int.Parse(ctLine[1]);
            int faceNum = int.Parse(inString[ptNum + cellNum + 2]);

            //start parsing
            double[] ptX = new double[ptNum];
            double[] ptY = new double[ptNum];
            double[] ptZ = new double[ptNum];

            for (int i = 0; i < ptNum; ++i)
            {
                
                string[] tmpSt = inString[i + 2].Split(' ');
                string[] wsRemovalStr = new string[3];
                int ct = 0;
                for (int j = 0; j < tmpSt.Length; ++j )
                {
                    if (tmpSt[j] != "" && ct<3)
                    {
                        wsRemovalStr[ct] = tmpSt[j];
                        ct++;
                    }
                }
                tmpSt = wsRemovalStr;
                //now fill the xyz of points
                if (dim == 2)
                {
                    ptX[i] = double.Parse(tmpSt[0]);
                    ptY[i] = double.Parse(tmpSt[1]);
                    ptZ[i] = 0;
                }
                else
                {
                    ptX[i] = double.Parse(tmpSt[0]);
                    ptY[i] = double.Parse(tmpSt[1]);
                    ptZ[i] = double.Parse(tmpSt[2]);
                }

            }

            //get cell areas (individual polygons)
            int[][] cellReg = new int[cellNum][];
            int[][] faceReg = new int[faceNum][];
            int[] fct = new int[faceNum];
            int[] cct = new int[cellNum];

            for (int i = 0; i < cellNum; ++i)
            {
                string[] tempSt = inString[i + 2 + ptNum].Split(' ');
                
                cct[i] = int.Parse(tempSt[0]);
                cellReg[i] = new int[cct[i]];
                
                for (int j = 0; j < cct[i]; ++j)
                {
                    //Feature.Print(tempSt[j]);
                    cellReg[i][j] = int.Parse(tempSt[j+1]);
                }

            }
            
            for (int i = 0; i < faceNum; ++i)
            {
                string[] tempSt = inString[i + 3 + ptNum + cellNum].Split(' ');
                
                //fct[i] = int.Parse(tempSt[0]);
                faceReg[i] = new int[dim];

                for (int j = 0; j < dim; ++j)
                {
                    //Feature.Print(tempSt[j]);
                    faceReg[i][j] = int.Parse(tempSt[j]);
                }

            }
            //if (isD3d)
            //{
            //    cellNum = faceNum;
            //    cellReg = faceReg;
            //    cct = fct;
            //}
            // make points

            DPoint3d[][] pt3d = new DPoint3d[cellNum][];
            
            for (int i = 0; i < cellNum; ++i)
            {
                pt3d[i] = new DPoint3d[cct[i]];
                
                for (int j = 0; j < cct[i]; ++j)
                {
                    pt3d[i][j] = new DPoint3d();
                    
                    if (dim == 2)
                    {
                        pt3d[i][j].X = ptX[cellReg[i][j]];
                        pt3d[i][j].Y = ptY[cellReg[i][j]];
                        pt3d[i][j].Z = 0;
                        
                    }
                    else
                    {
                        pt3d[i][j].X = ptX[cellReg[i][j]];
                        pt3d[i][j].Y = ptY[cellReg[i][j]];
                        pt3d[i][j].Z = ptZ[cellReg[i][j]];
                    }
                }
            }

            //filter infinity points for voranoi

            if (filterInf)
            {
                
                int ct = 0;
                for (int i = 0; i < pt3d.Length; ++i)
                {
                    int isInf = 0;
                    for (int j = 0; j < pt3d[i].Length; ++j)
                    {

                        if (pt3d[i][j].X == -10.101 && pt3d[i][j].Y == -10.101)
                        {
                            isInf = 1;

                        }
                    }
                    if (isInf == 0)
                    {
                        ct++;
                    }

                }
                 
                DPoint3d[][] tempPt = new DPoint3d[ct][];

                int ct2 = 0;
                int infCheck = 0;
                for (int i = 0; i < pt3d.Length; ++i)
                {
                    if (ct2 < ct)
                    {
                        tempPt[ct2] = new DPoint3d[pt3d[i].Length];

                        infCheck = 0;
                        for (int k = 0; k < pt3d[i].Length; ++k)
                        {
                            if (pt3d[i][k].X == -10.101 && pt3d[i][k].Y == -10.101)
                            {
                                infCheck = 1;
                            }
                        }
                        if (infCheck == 0)
                        {

                            for (int j = 0; j < pt3d[i].Length; ++j)
                            {
                                tempPt[ct2][j] = new DPoint3d();
                                tempPt[ct2][j] = pt3d[i][j];



                            }
                            ct2++;
                        }
                    }
                }
                pt3d = tempPt;
            }
            outPoints = pt3d;
        }

        public static void RunQhull(DPoint3d[] inputPoints, string qmodeString, int Dimension, ref String[] outInfo)
        {
            //fun starts here
            double ptNum = inputPoints.Length;

            //get qhull location 

            string homeDir = Bentley.GenerativeComponents.GCScript.Configuration.UserChoices.GCWorkspaceDirectory().ToString();

            string pSetCfgLoc = (homeDir + "rcQhull.cfg");

            //write type
            string qhCode = qmodeString + Dimension;

            string codeBase = Assembly.GetExecutingAssembly().CodeBase;
            UriBuilder uri = new UriBuilder(codeBase);
            string qhullLoc = Uri.UnescapeDataString(uri.Path);
            qhullLoc = Path.GetDirectoryName(qhullLoc);
            qhullLoc += @"\QHull\qhull.exe";


            string arg = qmodeString + " o i s";

            ProcessStartInfo psi = new ProcessStartInfo(qhullLoc);
            psi.UseShellExecute = false;
            psi.RedirectStandardInput = true;
            psi.RedirectStandardOutput = true;
            psi.CreateNoWindow = true;
            psi.Arguments = arg;

            Process strQhull;
            strQhull = Process.Start(psi);

            //process ptSet into qvoronoi format and create input file
            StreamWriter input = strQhull.StandardInput;

            //write headder
            input.WriteLine(Dimension);
            input.WriteLine(ptNum);

            //write point location

            if (Dimension == 3)
            {
                Random ran = new Random();
                for (int i = 0; i < ptNum; ++i)
                {
                    double jig = ran.NextDouble() * 0.001;
                    input.WriteLine((inputPoints[i].X + " " + inputPoints[i].Y + " " + (inputPoints[i].Z + jig)));

                }

            }
            if (Dimension == 2)
            {
                for (int i = 0; i < ptNum; ++i)
                {
                    input.WriteLine((inputPoints[i].X + " " + inputPoints[i].Y));

                }
            }
            input.Close();


            StreamReader output = strQhull.StandardOutput;
            Console.WriteLine(output);
            string outputStr = output.ReadToEnd();
            outInfo = outputStr.Split('\n');

        }

        public static DPoint3d[] orderConvexHull2d(DPoint3d[][] unOrderedPoints)
        {
            List<DPoint3d> orderedPoints = new List<DPoint3d>(unOrderedPoints.Length * 2);
            

            orderedPoints.AddRange(unOrderedPoints[0]);
            DPoint3d cp;   

            //for (int i = 1; i < unOrderedPoints.Length ; i++)
            while(orderedPoints.Count != unOrderedPoints.Length)
            {
                cp = orderedPoints[orderedPoints.Count - 1];
                for (int j = 1; j < unOrderedPoints.Length; j++)
                {
                    if (unOrderedPoints[j] != null)
                    {
                        if (unOrderedPoints[j][0].Distance(ref cp) < 0.001)
                        {
                            orderedPoints.Add(unOrderedPoints[j][1]);
                            unOrderedPoints[j] = null;
                            break;
                        }
                        if (unOrderedPoints[j][1].Distance(ref cp) < 0.001)
                        {
                            orderedPoints.Add(unOrderedPoints[j][0]);
                            unOrderedPoints[j] = null;
                            break;
                        }
                    }
                }
            }

            return orderedPoints.ToArray();
        }

    }


}