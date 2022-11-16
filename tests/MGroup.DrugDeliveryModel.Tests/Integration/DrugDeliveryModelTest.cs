using System;
using System.Collections.Generic;
using System.Linq;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Numerics.Interpolation.Jacobians;
using MGroup.DrugDeliveryModel.Tests.EquationModels;
using MGroup.NumericalAnalyzers.Dynamic;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.NumericalAnalyzers.Staggered;
using Xunit;

namespace MGroup.DrugDeliveryModel.Tests.Integration
{
	public class DrugDeliveryModelTest
	{
        const double Sc = 0.1;
        const double timeStep = 1; // in days
        const double totalTime = 10; // in days

        static double miNormal = 5; //KPa
        static double kappaNormal = 6.667; //Kpa
        static double miTumor = 22.44; //Kpa
        static double kappaTumor = 201.74; //Kpa
        static int currentTimeStep = 0;
        static double lambda0 = 1;
        static Dictionary<double, double[]> Solution = new Dictionary<double, double[]>(); private static List<(INode node, IDofType dof)> watchDofs = new List<(INode node, IDofType dof)>();

		public DrugDeliveryModelTest()
		{
            IsoparametricJacobian3D.DeterminantTolerance = 1e-20;
        }

        [Theory]
        [InlineData("../../../DataFiles/MeshCyprusTM.mphtxt")]
        public void MonophasicEquationModel(string fileName)
		{
			var equationModel = new MonophasicEquationModel(fileName, Sc, miNormal, kappaNormal, miTumor, kappaTumor, timeStep, totalTime, lambda0);
            var u1X = new double[(int)(totalTime / timeStep)];
            var u1Y = new double[(int)(totalTime / timeStep)];
            var u1Z = new double[(int)(totalTime / timeStep)];

            var staggeredAnalyzer = new StepwiseStaggeredAnalyzer(equationModel.ParentAnalyzers, equationModel.ParentSolvers, equationModel.CreateModel, maxStaggeredSteps: 3, tolerance: 1e-5);
            for (currentTimeStep = 0; currentTimeStep < totalTime / timeStep; currentTimeStep++)
            {
                equationModel.CurrentTimeStep = currentTimeStep;
                equationModel.CreateModel(equationModel.ParentAnalyzers, equationModel.ParentSolvers);
                staggeredAnalyzer.SolveCurrentStep();
                var allValues = ((DOFSLog)equationModel.ParentAnalyzers[0].ChildAnalyzer.Logs[0]).DOFValues.Select(x => x.val).ToArray();

                u1X[currentTimeStep] = allValues[0];
                u1Y[currentTimeStep] = allValues[1];
                u1Z[currentTimeStep] = allValues[2];

                if (Solution.ContainsKey(currentTimeStep))
                {
                    Solution[currentTimeStep] = allValues;
                    Console.WriteLine($"Time step: {timeStep}");
                    Console.WriteLine($"Displacement vector: {string.Join(", ", Solution[timeStep])}");
                }
                else
                {
                    Solution.Add(currentTimeStep, allValues);
                }

                for (int j = 0; j < equationModel.ParentAnalyzers.Length; j++)
                {
                    (equationModel.ParentAnalyzers[j] as PseudoTransientAnalyzer).AdvanceStep();
                }

                for (int j = 0; j < equationModel.ParentAnalyzers.Length; j++)
                {
                    equationModel.AnalyzerStates[j] = equationModel.ParentAnalyzers[j].CreateState();
                    equationModel.NLAnalyzerStates[j] = equationModel.NLAnalyzers[j].CreateState();
                }

                Console.WriteLine($"Displacement vector: {string.Join(", ", Solution[currentTimeStep])}");
            }
        }
    }
}
