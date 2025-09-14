using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using UnityEngine; // para JsonUtility

namespace Assets.Scripts
{
    public enum ActivationFunction
    {
        Identity,
        Sigmoid,
        Tanh,
        ReLU,
        Softmax
    }

    [Serializable]
    public class NeuralNetwork
    {
        private int inputNodes;
        private List<int> hiddenNodes;
        private int outputNodes;
        private List<Matrix> weights;
        private List<Matrix> biases;
        private double learningRate;

        private static readonly System.Random rand = new System.Random();

        public NeuralNetwork(int inputNodes, List<int> hiddenNodes, int outputNodes)
        {
            this.inputNodes = inputNodes;
            this.hiddenNodes = hiddenNodes;
            this.outputNodes = outputNodes;

            weights = new List<Matrix>();
            biases = new List<Matrix>();

            // Initialize weights and biases using He initialization for ReLU
            int previousLayerNodes = inputNodes;

            foreach (int layerSize in hiddenNodes)
            {
                double stdDev = Math.Sqrt(2.0 / previousLayerNodes);
                weights.Add(Matrix.Randomize(new Matrix(layerSize, previousLayerNodes), stdDev));
                biases.Add(Matrix.Randomize(new Matrix(layerSize, 1), stdDev));
                previousLayerNodes = layerSize;
            }

            // Add output layer
            double outputStdDev = Math.Sqrt(2.0 / previousLayerNodes);
            weights.Add(Matrix.Randomize(new Matrix(outputNodes, previousLayerNodes), outputStdDev));
            biases.Add(Matrix.Randomize(new Matrix(outputNodes, 1), outputStdDev));

            learningRate = 0.01;  // Ajuste utilizado anteriormente; mantido para compatibilidade
        }

        public double[] FeedForward(double[] inputsArray, ActivationFunction hiddenActivation = ActivationFunction.ReLU, ActivationFunction outputActivation = ActivationFunction.Tanh)
        {
            Matrix inputs = Matrix.FromArray(inputsArray);
            Matrix layerInput = inputs;

            for (int i = 0; i < weights.Count; i++)
            {
                // Calculate layer output
                Matrix layerOutput = Matrix.Multiply(weights[i], layerInput);
                layerOutput = Matrix.Add(layerOutput, biases[i]);

                var activation = i < weights.Count - 1 ? hiddenActivation : outputActivation;
                layerOutput.Map((x, row, col) => ActivationFunctionProvider.Activate(activation, x));

                layerInput = layerOutput; // Set the output as the next input
            }

            return Matrix.ToArray(layerInput);
        }

        public static NeuralNetwork Copy(NeuralNetwork nn)
        {
            NeuralNetwork copy = new NeuralNetwork(nn.inputNodes, new List<int>(nn.hiddenNodes), nn.outputNodes);
            copy.weights = nn.weights.Select(w => Matrix.Copy(w)).ToList();
            copy.biases = nn.biases.Select(b => Matrix.Copy(b)).ToList();
            copy.learningRate = nn.learningRate;
            return copy;
        }

        public static NeuralNetwork Crossover(NeuralNetwork parent1, NeuralNetwork parent2)
        {
            NeuralNetwork offspring = new NeuralNetwork(parent1.inputNodes, new List<int>(parent1.hiddenNodes), parent1.outputNodes);

            offspring.weights = new List<Matrix>();
            offspring.biases = new List<Matrix>();

            for (int i = 0; i < parent1.weights.Count; i++)
                offspring.weights.Add(Matrix.CrossoverMatrix(parent1.weights[i], parent2.weights[i]));
            for (int i = 0; i < parent1.biases.Count; i++)
                offspring.biases.Add(Matrix.CrossoverMatrix(parent1.biases[i], parent2.biases[i]));

            return offspring;
        }

        // Mutate with explicit gaussian std (mutação controlada por taxa e intensidade)
        public void Mutate(double mutationRate, double mutationStd)
        {
            foreach (var weight in weights)
            {
                weight.Map((elm, i, j) =>
                {
                    if (rand.NextDouble() < mutationRate)
                        return elm + rand.NextGaussian() * mutationStd;
                    return elm;
                });
            }
            foreach (var bias in biases)
            {
                bias.Map((elm, i, j) =>
                {
                    if (rand.NextDouble() < mutationRate)
                        return elm + rand.NextGaussian() * mutationStd;
                    return elm;
                });
            }
        }

        // Compatibilidade: chama usando learningRate como std (comportamento antigo)
        public void Mutate(double mutationRate)
        {
            Mutate(mutationRate, learningRate);
        }

        /* ===================== Serialização JSON ===================== */

        [Serializable]
        private class LayerDTO
        {
            public int rows;
            public int cols;
            public double[] data; // row-major
        }

        [Serializable]
        private class NNDTO
        {
            public int input;
            public int[] hidden;
            public int output;
            public LayerDTO[] weights;
            public LayerDTO[] biases;
            public double learningRate;
            public string note;
        }

        private static LayerDTO MatrixToDTO(Matrix m)
        {
            int rows = m.Rows;
            int cols = m.Cols;
            double[] flat = new double[rows * cols];
            int k = 0;
            for (int r = 0; r < rows; r++)
                for (int c = 0; c < cols; c++)
                    flat[k++] = m.data[r, c];

            return new LayerDTO { rows = rows, cols = cols, data = flat };
        }

        private static Matrix DTOToMatrix(LayerDTO dto)
        {
            Matrix m = new Matrix(dto.rows, dto.cols);
            int k = 0;
            for (int r = 0; r < dto.rows; r++)
                for (int c = 0; c < dto.cols; c++)
                    m.data[r, c] = dto.data[k++];
            return m;
        }

        public void SaveToJson(string path)
        {
            var dto = new NNDTO
            {
                input = inputNodes,
                hidden = hiddenNodes.ToArray(),
                output = outputNodes,
                weights = weights.Select(MatrixToDTO).ToArray(),
                biases = biases.Select(MatrixToDTO).ToArray(),
                learningRate = learningRate,
                note = "He init; activations: ReLU (hidden), Tanh (output)"
            };

            string json = JsonUtility.ToJson(dto, prettyPrint: true);
            File.WriteAllText(path, json, Encoding.UTF8);
        }

        public static NeuralNetwork LoadFromJson(string path)
        {
            if (!File.Exists(path)) return null;
            string json = File.ReadAllText(path, Encoding.UTF8);
            var dto = JsonUtility.FromJson<NNDTO>(json);
            if (dto == null) return null;

            var nn = new NeuralNetwork(dto.input, dto.hidden.ToList(), dto.output)
            {
                learningRate = dto.learningRate
            };
            nn.weights = dto.weights.Select(DTOToMatrix).ToList();
            nn.biases = dto.biases.Select(DTOToMatrix).ToList();
            return nn;
        }
    }

    public static class ActivationFunctionProvider
    {
        public static double Activate(ActivationFunction func, double x)
        {
            switch (func)
            {
                case ActivationFunction.Identity:
                    return x;
                case ActivationFunction.Sigmoid:
                    return 1.0 / (1.0 + Math.Exp(-x));
                case ActivationFunction.Tanh:
                    return Math.Tanh(x);
                case ActivationFunction.ReLU:
                    return Math.Max(0, x);
                case ActivationFunction.Softmax:
                    // Softmax é aplicado sobre um vetor – aqui é placeholder
                    return x;
                default:
                    return x;
            }
        }

        public static double Derivative(ActivationFunction func, double y)
        {
            switch (func)
            {
                case ActivationFunction.Identity:
                    return 1;
                case ActivationFunction.Sigmoid:
                    return y * (1 - y);
                case ActivationFunction.Tanh:
                    return 1 - y * y;
                case ActivationFunction.ReLU:
                    return y > 0 ? 1 : 0;
                default:
                    return 1;
            }
        }
    }
}
