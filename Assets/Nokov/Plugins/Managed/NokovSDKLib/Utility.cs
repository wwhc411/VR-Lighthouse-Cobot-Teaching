// ***********************************************************************
// Assembly         : Assembly-CSharp
// Author           : duguguang
// Created          : 07-26-2020
//
// Last Modified By : duguguang
// Last Modified On : 07-26-2020
// ***********************************************************************
// <copyright file="Utility.cs" company="Nokov">
//     Copyright (c) Nokov. All rights reserved.
// </copyright>
// <summary>Function Class</summary>
// ***********************************************************************

using System;
using System.Collections.Generic;
using System.Data;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


namespace Utility
{
    internal class SmoothFrameArray
    {
        public  const int DefaultSize = 7;
        public SmoothFrameArray(int size = DefaultSize)
        {
            _arraySize = size;
            _list = new LinkedList<double>();
        }

        public int Mid(int size)
        {
            return (int)size / 2;
        }

        public void clear()
        {
            _list.Clear();
        }

        public int arraySize()
        {
            return _arraySize;
        }

        public int cache(double data)
        {
            _list.AddLast(data);
            return _list.Count();
        }

        public bool full()
        {
            return _list.Count() >= _arraySize;
        }

        public bool frameArray(ref double[] retArray)
        {
            if(!full())
                return false;


            retArray = new double[_arraySize];

            for (int i = 0; i < _list.Count(); ++i)
            {
                retArray[i] = _list.ElementAt(i);
            }

            _list.RemoveFirst();

            return true;

        }

        public  bool tryToSmooth(ref float data)
        {
            double[] inArray = null;
            if (!frameArray(ref inArray))
            {
                if (inArray != null)
                {
                    inArray = null;
                }

                return false;
            }

            double[] outArray = new double[_arraySize];
            int retCount = average7(inArray, ref outArray, _arraySize);
            if (retCount != 0)
            {
                if (outArray != null)
                {
                    inArray = null;
                    outArray = null;
                }

                return false;
            }

            data = (float)outArray[Mid(_arraySize)];

            inArray = null;
            outArray = null;

            return true;
        }


        protected int average7(double[] inArray, ref double[] outArray, int N)
        {
            if (null == inArray || null == outArray)
                return 1;

            if (N <= 0)
                return 1;

            if (N < 7)
            {
                for (int i = 0; i <= N; i++)
                    outArray[i]= inArray[i];
            }
            else
            {
                outArray[0]= inArray[0];
                outArray[1]= inArray[1];
                outArray[2]= inArray[2];
                for (int i = 3; i < N - 3; i++)
                {
                    outArray[i] = (inArray[i - 3] + inArray[i - 2] + inArray[i - 1] + inArray[i] + inArray[i + 1] + inArray[i + 2] + inArray[i + 3]) / 7;
                }

                outArray[N - 3] = inArray[N - 3];
                outArray[N - 2] = inArray[N - 2];
                outArray[N - 1] = inArray[N - 1];
            }

	         return 0;
        }

        private LinkedList<double> _list;
        private int _arraySize;
    }

    internal class SmoothFramePointArray
    {
        public SmoothFramePointArray(int size = SmoothFrameArray.DefaultSize)
        {
            SmoothFrameArray _xArray = new SmoothFrameArray(size);
            SmoothFrameArray _yArray = new SmoothFrameArray(size);
            SmoothFrameArray _zArray = new SmoothFrameArray(size);
        }

        public void Reset()
        {
            _xArray.clear();
            _yArray.clear();
            _zArray.clear();
        }

        public void Cache(double x, double y, double z)
        {
            _xArray.cache(x);
            _yArray.cache(y);
            _zArray.cache(z);
        }

        public void Smooth(ref float x, ref float y , ref float z)
        {
            _xArray.tryToSmooth(ref x);
            _yArray.tryToSmooth(ref y);
            _zArray.tryToSmooth(ref z);
        }

        private SmoothFrameArray _xArray = new SmoothFrameArray();
        private SmoothFrameArray _yArray = new SmoothFrameArray();
        private SmoothFrameArray _zArray = new SmoothFrameArray();
    }

}

