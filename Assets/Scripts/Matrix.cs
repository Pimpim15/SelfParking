using System;
using System.Globalization;
using System.Linq;
using UnityEngine;
using Random = System.Random;

namespace Assets.Scripts
{
    public class Matrix
    {
        public int Rows { get; private set; }
        public int Cols { get; private set; }
        public double[,] data { get; private set; }

        private static readonly Random rand = new Random();

        public Matrix(int rows, int cols)
        {
            Rows = rows;
            Cols = cols;
            data = new double[rows, cols];
        }

        public static Matrix Map(Matrix A, Func<double, int, int, double> func)
        {
            var matrix = new Matrix(A.Rows, A.Cols);
            for (int i = 0; i < A.Rows; i++)
                for (int j = 0; j < A.Cols; j++)
                    matrix.data[i, j] = func(A.data[i, j], i, j);

            return matrix;
        }

        public void Map(Func<double, int, int, double> func)
        {
            for (int i = 0; i < Rows; i++)
                for (int j = 0; j < Cols; j++)
                    data[i, j] = func(data[i, j], i, j);
        }

        public static Matrix Randomize(Matrix m, double stdDev = 1)
        {
            return Map(m, (elm, i, j) => rand.NextGaussian(0, stdDev));
        }

        public static string MatrixToString(Matrix m)
        {
            return string.Join(";", Matrix.ToArray(m).AsEnumerable().Select(x => x.ToString(CultureInfo.InvariantCulture)).ToArray());
        }

        public static Matrix ParseMatrix(string line, int rows = 0, int cols = 0)
        {
            var parts = line.Split(';', StringSplitOptions.RemoveEmptyEntries).Skip(1);
            double[] values = parts.Select(x => double.Parse(x.Trim(), CultureInfo.InvariantCulture)).ToArray();

            if (rows == 0 || cols == 0)
                return Matrix.FromArray(values);
            else
            {
                Matrix m = new Matrix(rows, cols);
                int index = 0;
                return Map(m, (elm, i, j) => values[index++]);
            }
        }

        public static Matrix CrossoverMatrix(Matrix matrix1, Matrix matrix2)
        {
            Matrix newMatrix = new Matrix(matrix1.Rows, matrix1.Cols);

            return Map(newMatrix, (elm, i, j) => rand.NextDouble() < 0.5 ? matrix1.data[i, j] : matrix2.data[i, j]);
        }

        public static Matrix FromArray(double[] arr)
        {
            var matrix = new Matrix(arr.Length, 1);

            for (int i = 0; i < arr.Length; i++)
                matrix.data[i, 0] = arr[i];

            return matrix;
        }

        public static double[] ToArray(Matrix matrix)
        {
            double[] arr = new double[matrix.Rows * matrix.Cols];
            int index = 0;

            for (int i = 0; i < matrix.Rows; i++)
                for (int j = 0; j < matrix.Cols; j++)
                    arr[index++] = matrix.data[i, j];

            return arr;
        }

        public void Print(string name = "")
        {
            string log = name + "\n";

            for (int i = 0; i < Rows; i++)
            {
                for (int j = 0; j < Cols; j++)
                {
                    log += $"[{i},{j}] = {data[i, j]} ";
                }
                log += "\n";
            }

            Debug.Log(log);
        }

        public static Matrix Transpose(Matrix A)
        {
            var matrix = new Matrix(A.Cols, A.Rows);

            for (int i = 0; i < A.Rows; i++)
                for (int j = 0; j < A.Cols; j++)
                    matrix.data[j, i] = A.data[i, j];

            return matrix;
        }

        public static Matrix MultiplyScalar(Matrix A, double scalar)
        {
            return Map(A, (elm, i, j) => elm * scalar);
        }

        public static Matrix Hadamard(Matrix A, Matrix B)
        {
            return Map(A, (elm, i, j) => A.data[i, j] * B.data[i, j]);
        }

        public static Matrix Add(Matrix A, Matrix B)
        {
            return Map(A, (elm, i, j) => A.data[i, j] + B.data[i, j]);
        }

        public static Matrix Subtract(Matrix A, Matrix B)
        {
            return Map(A, (elm, i, j) => A.data[i, j] - B.data[i, j]);
        }

        public static Matrix Multiply(Matrix A, Matrix B)
        {
            if (A.Cols != B.Rows)
                throw new Exception($"Incompatible matrix dimensions. A[{A.Rows},{A.Cols}], B[{B.Rows},{B.Cols}]");

            Matrix result = new Matrix(A.Rows, B.Cols);

            for (int i = 0; i < result.Rows; i++)
                for (int j = 0; j < result.Cols; j++)
                {
                    double sum = 0;
                    for (int k = 0; k < A.Cols; k++)
                        sum += A.data[i, k] * B.data[k, j];
                    result.data[i, j] = sum;
                }

            return result;
        }

        public static Matrix Copy(Matrix m)
        {
            Matrix copy = new Matrix(m.Rows, m.Cols);

            for (int i = 0; i < m.Rows; i++)
                for (int j = 0; j < m.Cols; j++)
                    copy.data[i, j] = m.data[i, j];

            return copy;
        }

        public static Matrix Softmax(Matrix input)
        {
            if (input.Cols != 1)
                throw new ArgumentException("Softmax function is designed to work with a single column matrix.");

            // Stabilize the inputs to prevent overflow
            double maxVal = input.data.Cast<double>().Max();

            Matrix exps = new Matrix(input.Rows, 1);
            double sumExps = 0.0;

            for (int i = 0; i < input.Rows; i++)
            {
                exps.data[i, 0] = Math.Exp(input.data[i, 0] - maxVal);
                sumExps += exps.data[i, 0];
            }

            for (int i = 0; i < input.Rows; i++)
            {
                exps.data[i, 0] /= sumExps;
            }

            return exps;
        }
    }

    public static class RandomExtensions
    {
        private static double spare;
        private static bool hasSpare = false;

        // Generates a normally distributed random number using Box-Muller transform
        public static double NextGaussian(this Random rand, double mean = 0, double stdDev = 1)
        {
            if (hasSpare)
            {
                hasSpare = false;
                return spare * stdDev + mean;
            }

            double u, v, s;
            do
            {
                u = rand.NextDouble() * 2 - 1;
                v = rand.NextDouble() * 2 - 1;
                s = u * u + v * v;
            } while (s >= 1 || s == 0);

            s = Math.Sqrt(-2.0 * Math.Log(s) / s);
            spare = v * s;
            hasSpare = true;
            return mean + stdDev * u * s;
        }
    }
}
