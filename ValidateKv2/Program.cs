using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.Xbox.Input.Nui;
using Microsoft.Kinect;



namespace ReadDepthDataToPointCloud
{
    class Program
    {
        static void Main(string[] args)
        {
            // get the coordinate mapper
            KinectSensor kinectSensor = KinectSensor.GetDefault();
            PointF[] pf = kinectSensor.CoordinateMapper.GetDepthFrameToCameraSpaceTable();

            List<Tuple<int, string>> subjectIdList = new List<Tuple<int, string>>();
            List<string> xefFileNameList = new List<string>();

            //process exception
            xefFileNameList.Add(@"E:\PiGKinectValidation\Female\PKV03\Kinect Validation\Y-Excrusion\YE01.xef"); subjectIdList.Add(new Tuple<int, string>(3, "YE01"));
            xefFileNameList.Add(@"E:\PiGKinectValidation\Female\PKV07\Kinect Validation\Y-Excrusion\YE01.xef"); subjectIdList.Add(new Tuple<int, string>(7, "YE01"));
           
            for (int n = 0; n < xefFileNameList.Count; n++)
            {

                string xefFileName = xefFileNameList[n];

                XefFileReader reader = new XefFileReader();
                XefFileReader.ErrorCode err = reader.Load(xefFileName);
                uint frameNum = reader.GetNumFrames();
                uint startFrame = reader.GetStartFrame();
                List<ushort[]> dimgSet = new List<ushort[]>();
                float[] camerapara = reader.GetCalibrationData();

                Console.WriteLine(xefFileName);

                for (uint nf = 0; nf < frameNum; nf++)
                {
                    ulong timeStamp = 0;
                    ushort[] dimg = reader.GetDepthFrame(startFrame + nf, out timeStamp);
                    dimgSet.Add(dimg);
                    BODY_FRAME bodyFrame = reader.GetSkeletonFrame(startFrame + nf, out timeStamp, true);
                    int bodyIndex = 0;
                    BODY_DATA bd = new BODY_DATA();

                    if (bodyFrame.BodyData.body0.TrackingState == BODY_TRACKING_STATE.TRACKED) { bodyIndex = 0; bd = bodyFrame.BodyData.body0; }
                    if (bodyFrame.BodyData.body1.TrackingState == BODY_TRACKING_STATE.TRACKED) { bodyIndex = 1; bd = bodyFrame.BodyData.body1; }
                    if (bodyFrame.BodyData.body2.TrackingState == BODY_TRACKING_STATE.TRACKED) { bodyIndex = 2; bd = bodyFrame.BodyData.body2; }
                    if (bodyFrame.BodyData.body3.TrackingState == BODY_TRACKING_STATE.TRACKED) { bodyIndex = 3; bd = bodyFrame.BodyData.body3; }
                    if (bodyFrame.BodyData.body4.TrackingState == BODY_TRACKING_STATE.TRACKED) { bodyIndex = 4; bd = bodyFrame.BodyData.body4; }
                    if (bodyFrame.BodyData.body5.TrackingState == BODY_TRACKING_STATE.TRACKED) { bodyIndex = 5; bd = bodyFrame.BodyData.body5; }

                    Tuple<Single, Single, Single, Single> floorClip = new Tuple<float, float, float, float>(bodyFrame.FloorClipPlane.x,
                                                                                          bodyFrame.FloorClipPlane.y,
                                                                                          bodyFrame.FloorClipPlane.z,
                                                                                          bodyFrame.FloorClipPlane.w);

                    YEDataFrame YEframe = new YEDataFrame(dimg, bd, floorClip, nf, subjectIdList[n].Item1, subjectIdList[n].Item2, camerapara);
                    YEframe.Process();
                    Console.WriteLine(nf.ToString());
                }

                

                dimgSet.Clear();
            }

            //Console.ReadLine(); //Pause
            return;
        }
    }
}
