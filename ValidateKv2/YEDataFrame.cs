using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Threading.Tasks;
using Microsoft.Xbox.Input.Nui;
using Microsoft.Kinect;

namespace ReadDepthDataToPointCloud
{
    class YEDataFrame
    {
        const int DIMGSIZE = 512 * 424;
        ushort[] m_depth = new ushort[DIMGSIZE];
        BODY_DATA m_body;
        Tuple<Single, Single, Single, Single> m_floor;
        int m_subjectID;
        string m_trialID;
        uint m_frm;
        float[] m_para = new float[DIMGSIZE * 2 + 33];
        float[] m_pc = new float[DIMGSIZE * 3];

        float[] m_pcOriginCamera = new float[DIMGSIZE * 3];
        float[] m_pcOriginAdjusted = new float[DIMGSIZE * 3];
        float[] m_pcSubjectCamera = new float[DIMGSIZE * 3];
        float[] m_pcSubjectAdjusted = new float[DIMGSIZE * 3];

        public YEDataFrame(ushort[] dimg, BODY_DATA bd, Tuple<Single, Single, Single, Single> floorClip, uint num, int subjectID, string trialID, float[] para)
        {
            m_depth = dimg;
            m_body = bd;
            m_floor = floorClip;
            m_frm = num;
            m_subjectID = subjectID;
            m_trialID = trialID;
            m_para = para;
        }

        ~YEDataFrame()
        {
            m_para = null;
            m_depth = null;
            m_pc = null;

            m_pcOriginCamera = null;
            m_pcOriginAdjusted = null;
            m_pcSubjectCamera = null;
            m_pcSubjectAdjusted = null;
        }

        public void Process()
        {
            GeneratePointCloudFromDepth(m_depth, m_para);
            SavePointCloud();
        }

        public void GeneratePointCloudFromDepth(ushort[] depth, float[] para)
        {
            float[] pc = new float[DIMGSIZE * 3];
            for (int i = 0; i < DIMGSIZE; i++)
            {
                m_pc[3*i] = depth[i] * para[6+i*2] / 1000;
                m_pc[3 * i + 1] = depth[i] * para[6 + i * 2 + 1] / 1000;
                m_pc[3 * i + 2] = depth[i] * 1.0f / 1000;

                m_pcOriginCamera[3 * i] = m_pc[3 * i];
                m_pcOriginCamera[3 * i + 1] = m_pc[3 * i + 1];
                m_pcOriginCamera[3 * i + 2] = m_pc[3 * i + 2];
            }

            m_pcOriginAdjusted = FloorPlaneAdjustment(m_pcOriginCamera, m_floor);
            m_pcSubjectAdjusted = BackgroundSubtraction(m_pcOriginAdjusted);
        }

        public void SavePointCloud()
        {
            string storageRootDir = "E:\\YExcusionPointCloud";

            string path1 = storageRootDir + "\\PK" + m_subjectID.ToString() + "_" + m_subjectID.ToString() + "\\OriginalCamera";
            if (!Directory.Exists(path1))
            {
                Directory.CreateDirectory(path1);
            }
            string frameFileName = path1 + "\\" + m_frm.ToString() + ".txt";
            using (StreamWriter writer = new StreamWriter(frameFileName))
            {
                foreach (var value in m_pcOriginCamera)
                {
                    writer.WriteLine(value);
                }
            }

            string path2 = storageRootDir + "\\PK" + m_subjectID.ToString() + "_" + m_subjectID.ToString() + "\\OriginalAdjusted";
            if (!Directory.Exists(path2))
            {
                Directory.CreateDirectory(path2);
            }
            string frameFileName2 = path2 + "\\" + m_frm.ToString() + ".txt";
            using (StreamWriter writer = new StreamWriter(frameFileName2))
            {
                foreach (var value in m_pcOriginAdjusted)
                {
                    writer.WriteLine(value);
                }
            }

            string path3 = storageRootDir + "\\PK" + m_subjectID.ToString() + "_" + m_subjectID.ToString() + "\\SubjectAdjusted";
            if (!Directory.Exists(path3))
            {
                Directory.CreateDirectory(path3);
            }
            string frameFileName3 = path3 + "\\" + m_frm.ToString() + ".txt";
            using (StreamWriter writer = new StreamWriter(frameFileName3))
            {
                foreach (var value in m_pcSubjectAdjusted)
                {
                    writer.WriteLine(value);
                }
            }
        }

        public float[] BackgroundSubtraction(float[] src)
        {
            List<float> p = new List<float>();

            float[] spinbaseCamera = { m_body.SkeletonJointPositions.SPINE_BASE.x, m_body.SkeletonJointPositions.SPINE_BASE.y, m_body.SkeletonJointPositions.SPINE_BASE.z };
            float[] spinbaseAdjusted = FloorPlaneAdjustment(spinbaseCamera, m_floor);

            for (int i = 0; i < src.Length / 3; i++ )
            {
                if (Math.Abs(src[3 * i] - spinbaseAdjusted[0]) < 1.5f && src[3 * i + 1] > 0 && src[3 * i + 1] < 2 && Math.Abs(src[3 * i + 2] - spinbaseAdjusted[2]) < 1.5f)
                {
                    p.Add(src[3 * i]);
                    p.Add(src[3 * i+1]);
                    p.Add(src[3 * i+2]);
                }
            }


            return p.ToArray();
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="skeleton1"></param>
        /// <param name="floorClip"></param>
        /// <returns></returns>
        public float[] FloorPlaneAdjustment(float[] src, Tuple<Single, Single, Single, Single> floorClip)
        {
            float[] res = new float[src.Length];
            //FOR ERIK: what does xd stand for? first vector
            var vector1 = new Single[3];
            vector1[0] = floorClip.Item1;
            vector1[1] = floorClip.Item2;
            vector1[2] = floorClip.Item3;

            //FOR ERIK: what does x2 stand for? second vector that defines a coordinate system with respect to the ground plane
            var vector2 = new Single[3];
            vector2[0] = 0;
            vector2[1] = vector1[0];
            vector2[2] = vector1[1];

            //FOR ERIK: what does vmag stand for? vertical magnitude
            var verticalMagnitude = Math.Sqrt((vector2[1] * vector2[1]) + (vector2[2] * vector2[2]));
            vector2[0] = Convert.ToSingle(vector2[0] / verticalMagnitude);
            vector2[1] = Convert.ToSingle(vector2[1] / verticalMagnitude);
            vector2[2] = Convert.ToSingle(vector2[2] / verticalMagnitude);

            //FOR ERIK: what does x3 stand for? third vector
            var vector3 = new Single[3];
            vector3[0] = vector1[1] * vector2[2] - vector1[2] * vector2[1];
            vector3[1] = vector1[2] * vector2[0] - vector1[0] * vector2[2];
            vector3[2] = vector1[0] * vector2[1] - vector1[1] * vector2[0];
            if (vector3[0] < 0)
            {
                //Need it to go positive x...
                vector3[0] = -vector3[0];
                vector3[1] = -vector3[1];
                vector3[2] = -vector3[2];
            }

            //FOR ERIK: what does rData stand for? rotation matrix data
            var rotationMatData = new Single[9];
            //Right
            rotationMatData[0] = vector3[0];
            rotationMatData[1] = vector3[1];
            rotationMatData[2] = vector3[2];
            //Up, since Y is up
            rotationMatData[3] = vector1[0];
            rotationMatData[4] = vector1[1];
            rotationMatData[5] = vector1[2];
            //Forward
            rotationMatData[6] = vector2[0];
            rotationMatData[7] = vector2[1];
            rotationMatData[8] = vector2[2];

            //FOR ERIK: what does nj stand for? number of joints
            var jointCount = Enum.GetNames(typeof(JointType)).Length;

            for (var i = 0; i < src.Length / 3; i++)
            {
                float[] pt = new float[3];
                AdjustJoint(pt, src[3 * i], src[3 * i + 1], src[3 * i + 2], rotationMatData, floorClip.Item4);
                res[3 * i] = pt[0];
                res[3 * i + 1] = pt[1];
                res[3 * i + 2] = pt[2];
            }
            //Console.WriteLine("floorclip[4]={0}", floorClip.Item4);

            //Console.WriteLine("rdata = {0} {1} {2} {3} {4} {5} {6} {7} {8}", rData[0], rData[1], rData[2], rData[3], rData[4], rData[5], rData[6], rData[7], rData[8]);
            return res;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="j"></param>
        /// <param name="rData"></param>
        /// <param name="fY"></param>
        /// <returns></returns>
        public static void AdjustJoint(float[] pt, float xx, float yy, float zz, Single[] rData, Single fY)
        {

            var x = rData[0] * xx + rData[1] * yy + rData[2] * zz;
            var y = rData[3] * xx + rData[4] * yy + rData[5] * zz + fY;
            var z = rData[6] * xx + rData[7] * yy + rData[8] * zz;

            //FOR ERIK: what does tj stand for? temporary joint or new joint
            pt[0] = x;
            pt[1] = y;
            pt[2] = z;

        }


    }
}
