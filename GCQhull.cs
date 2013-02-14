using System;
using System.Text;
using System.Text.RegularExpressions;
using System.IO;
using System.Diagnostics;
using System.Collections.Generic;
using System.ComponentModel;
using Bentley.Geometry;
using Bentley.GenerativeComponents;
using Bentley.GenerativeComponents.Features.Specific;
using Bentley.GenerativeComponents.MicroStation;
using Bentley.Interop.MicroStationDGN;
using Bentley.GenerativeComponents.GCScript;
using Bentley.GenerativeComponents.GCScript.Functions;
using Bentley.GenerativeComponents.XceedHelpers;
using QhullHelper;



namespace Bentley.GenerativeComponents.Features // Must be in this namespace.
{


    public enum QhullCommand { Voronoi2D, Voronoi3D, ConvexHull2D, ConvexHull3D, Delaunay2D, Delaunay3D, Custom }
    [HideInheritedTechniques]
    public class Qhull: Polygon
    {


        /*------------------------------------------------------------------------------------**/
        /// <summary>Link qhull with GC</summary>
        /// <author>RJC</author>                                        
        /// <date>2008</date>

        /*--------------+---------------+---------------+---------------+---------------+------*/
   
       
        [Technique, ByFunction]
        public override bool ByFunction
        (
        FeatureUpdateContext updateContext,
        Function Function,
        IGCObject[] FunctionArguments
        )
        {
            return base.ByFunction(updateContext, Function, FunctionArguments);
        }

        #region qhull update method
        //  /*
        

        [DefaultTechnique]
        public bool QhullConnect
            (
            FeatureUpdateContext updateContext,
            [ Replicatable]                                 IPoint[] inputPoints,
            [Replicatable]                                      QhullCommand qmode, 
            [Replicatable,Optional]                             string customString,
            [Replicatable,Optional, DefaultValue(true)]        bool createPolygon,
            [Out]                                               ref Point[][] Points,
            [Out]                                               ref Polygon[][] vor3dPolygon,
            [Out]                                               ref Polygon[] polygonOut
           
            
            )
        {
                DPoint3d[] ipToDp = Translation.ConvertToDPoint3dArray(inputPoints);
           
                DeleteConstituentFeatures(updateContext);
                this.LetConstituentFeaturesBeDirectlyIndexible();
                LetConstituentFeaturesBeDirectlyIndexible();
                base.LetConstituentFeaturesBeDirectlyIndexible();

                //parse enum
                string qmodeString = "";
                int Dimension = 0;
                //fill outs
                vor3dPolygon = null;
                polygonOut = null;
                
                // var to check if delaunay 3d
                bool isD3d = false;

                bool filterInf = false;
                if (qmode == QhullCommand.ConvexHull2D)
                {
                    qmodeString = "c";
                    Dimension = 2;
                    filterInf = false;
                }
                if (qmode == QhullCommand.ConvexHull3D)
                {
                    qmodeString = "c";
                    Dimension = 3;
                    filterInf = false;
                }
                if (qmode == QhullCommand.Delaunay2D)
                {
                    qmodeString = "d Qbb";
                    Dimension = 2;
                    filterInf = false;
                }
               if (qmode == QhullCommand.Delaunay3D)
                {
                  qmodeString = "d";
                   Dimension = 3;
                   isD3d = true;
               }
                if (qmode == QhullCommand.Voronoi2D)
                {
                    qmodeString = "v Qbb";
                    Dimension = 2;
                    filterInf = true;
                    
                }
                if (qmode == QhullCommand.Voronoi3D)
                {
                    qmodeString = "v Qj";
                    Dimension = 3;
                    filterInf = true;

                }
                if (qmode == QhullCommand.Custom)
                {
                    string[] qms = Regex.Split(customString, ",");
                    int cDim = int.Parse(qms[0]);
                    qmodeString = qms[1];
                    Dimension = cDim;
                    filterInf = true;

                }
                
                // run qhull
                string[] outputStrSp = null;
                QhullUtility.RunQhull(ipToDp, qmodeString, Dimension, ref outputStrSp);
                
                DPoint3d[][] dpoints; 
                //parse the output
                QhullUtility.OffParser(outputStrSp, Dimension, filterInf, isD3d, out dpoints);
                //OUT POINTS
                Point[][] p = new Point[dpoints.Length][];
                for (int i = 0; i < dpoints.Length; i++ )
                {
                    p[i] = new Point[dpoints[i].Length];

                    for (int j = 0; j < dpoints[i].Length; j++)
                    {
                        CoordinateSystem cs = this.ParentModel().BaseCoordinateSystem();
                        p[i][j] = new Point(this);
                        p[i][j].FromDPoint3d(updateContext, cs, dpoints[i][j]);
                        p[i][j].SetSuccess(true);


                    }

                }
                Points = p;
         
                //take care of 3d vor by finding convex hull of each region

                DPoint3d[][] dpRefTemp = dpoints;

                if (createPolygon)
                {
                    if (qmode == QhullCommand.Voronoi3D)
                    {
                        string[] tempOut = null;
                        DPoint3d[][] tempDp = null;

                        Polygon[][] tempPg = new Polygon[dpRefTemp.Length][];
                        createPolygon = false;
                        int ct = 0;

                        for (int i = 0; i < dpoints.Length; i++)
                        {

                            QhullUtility.RunQhull(dpRefTemp[i], "c", 3, ref tempOut);
                            if (tempOut.Length >= 3)
                            {
                                QhullUtility.OffParser(tempOut, 3, true, false, out tempDp);

                                //cereate polygon
                                tempPg[ct] = new Polygon[tempDp.Length];
                                for (int j = 0; j < tempDp.Length; ++j)
                                {
                                    tempPg[ct][j] = new Polygon(this);
                                    tempPg[ct][j].AlignOptions(this);
                                    tempPg[ct][j].DenoteAsHavingBeenUpdated();
                                    tempPg[ct][j].ByVerticesAsDPoint3ds(updateContext, tempDp[j], false, false, 0, null, null);
                                    tempPg[ct][j].SetSuccess(true);

                                    AddConstituentFeature(tempPg[ct][j]);

                                }
                                ct++;
                            }


                        }

                        vor3dPolygon = tempPg;

                        return true;
                    }
                    else if (qmode == QhullCommand.ConvexHull2D)
                    {

                        List<Polygon> tempPg = new List<Polygon>(1);
                        Polygon pg = new Polygon(this);
                        pg.ByVerticesAsDPoint3ds(updateContext, QhullUtility.orderConvexHull2d(dpRefTemp), false, false, 1, null, null);
                        pg.SetSuccess(true);
                        tempPg.Add(pg);
                        polygonOut = tempPg.ToArray();
                        this.AddConstituentFeatures(polygonOut);


                        return true;
                    }
                    else
                    {
                        List<Polygon> tempPg = new List<Polygon>(dpRefTemp.Length);

                        for (int j = 0; j < dpRefTemp.Length; j++)
                        {
                            if (dpRefTemp[j].Length > 2)
                            {
                                Polygon pg = new Polygon(this);
                                pg.ByVerticesAsDPoint3ds(updateContext, dpRefTemp[j], false, false, 1, null, null);
                                pg.SetSuccess(true);
                                tempPg.Add(pg);
                            }

                        }

                        polygonOut = tempPg.ToArray();
                        this.AddConstituentFeatures(polygonOut);

                        return true;
                    }
                }

                return false;

        }
        // */
        #endregion

      
    
    } // class

} // namespace