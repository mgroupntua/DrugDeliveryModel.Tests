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
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MGroup.Constitutive.Structural;
using MGroup.DrugDeliveryModel.Tests.Commons;
using MGroup.NumericalAnalyzers;
using MGroup.Solvers.Direct;
using Xunit;
using MGroup.Constitutive.ConvectionDiffusion;

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
        //[InlineData("../../../DataFiles/workingTetMesh4886.mphtxt")]
        //[InlineData("../../../DataFiles/chipMelter2M.mphtxt")]
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

        [Fact]
        public void StaticLinearTestOrthogonalParallelepiped()
        {
            var model = Utilities.GetParallelepipedMesh();

            var solverFactory = new SkylineSolver.Factory();
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemStructural(model, algebraicModel, solver);
            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
            var staticAnalyzer = new StaticAnalyzer(algebraicModel, problem, linearAnalyzer);

            staticAnalyzer.Initialize();
            staticAnalyzer.Solve();
        }

        [Theory]
        [InlineData("../../../DataFiles/sanityCheckMesh.mphtxt")]
        public void StaticLinearTest(string fileName)
        {
            var equationModel = new MonophasicEquationModel(fileName, Sc, miNormal, kappaNormal, miTumor, kappaTumor, timeStep, totalTime, lambda0);
            Dictionary<int, double> lambda = new Dictionary<int, double>(equationModel.Reader.ElementConnectivity.Count());
            foreach (var elem in equationModel.Reader.ElementConnectivity)
            {
                lambda.Add(elem.Key, elem.Value.Item3 == 0 ? equationModel.CalculateLambda(currentTimeStep * timeStep) : 1d);
            }
            var model = new Model[] { EquationModels.MonophasicEquationModel.CreateElasticModelFromComsolFile(equationModel.Reader, miNormal, kappaNormal, miTumor, kappaTumor, lambda), };
            var solverFactory = new SkylineSolver.Factory() { FactorizationPivotTolerance = 1e-8 };
            var algebraicModel = new[] { solverFactory.BuildAlgebraicModel(model[0]), };
            var solver = new[] { solverFactory.BuildSolver(algebraicModel[0]), };
            var problem = new[] { new ProblemStructural(model[0], algebraicModel[0], solver[0]), };
            var linearAnalyzer = new LinearAnalyzer(algebraicModel[0], solver[0], problem[0]);
            var analyzer = new StaticAnalyzer( algebraicModel[0], problem[0], linearAnalyzer);
            analyzer.Initialize();
            analyzer.Solve();
        }

        [Theory]
        [InlineData("../../../DataFiles/NotSoSimpleTetMesh.mphtxt")]
        public void StaticNonLinearTest(string fileName)
        {
            var equationModel = new MonophasicEquationModel(fileName, Sc, miNormal, kappaNormal, miTumor, kappaTumor, timeStep, totalTime, lambda0);
            Dictionary<int, double> lambda = new Dictionary<int, double>(equationModel.Reader.ElementConnectivity.Count());
            currentTimeStep = 0;
            foreach (var elem in equationModel.Reader.ElementConnectivity)
            {
                lambda.Add(elem.Key, elem.Value.Item3 == 0 ? equationModel.CalculateLambda(currentTimeStep * timeStep) : 1d);
            }
            var model = new Model[] { EquationModels.MonophasicEquationModel.CreateElasticModelFromComsolFile(equationModel.Reader, miNormal, kappaNormal, miTumor, kappaTumor, lambda), };
            var solverFactory = new SkylineSolver.Factory() { FactorizationPivotTolerance = 1e-8 };
            var algebraicModel = new[] { solverFactory.BuildAlgebraicModel(model[0]), };
            var solver = new[] { solverFactory.BuildSolver(algebraicModel[0]), };
            var problem = new[] { new ProblemStructural(model[0], algebraicModel[0], solver[0]), };
            var loadControlAnalyzerBuilder = new LoadControlAnalyzer.Builder( algebraicModel[0], solver[0], problem[0], 1)
            {
                ResidualTolerance = 1E-7,
                MaxIterationsPerIncrement = 20,
                NumIterationsForMatrixRebuild = 1
            };
            var loadControlAnalyzer = loadControlAnalyzerBuilder.Build();
            loadControlAnalyzer.TotalDisplacementsPerIterationLog = new TotalDisplacementsPerIterationLog(
                new List<(INode node, IDofType dof)>()
                {
                            (model[0].NodesDictionary[333], StructuralDof.TranslationX),
                            (model[0].NodesDictionary[333], StructuralDof.TranslationY),
                            (model[0].NodesDictionary[333], StructuralDof.TranslationZ),

                }, algebraicModel[0]
            );
            var analyzer = new StaticAnalyzer(algebraicModel[0], problem[0], loadControlAnalyzer);
            analyzer.Initialize();
            analyzer.Solve();
            //var u1X = ((DOFSLog)parentAnalyzers[0].ChildAnalyzer.Logs[0]).DOFValues.FirstOrDefault().val;
        }

        [Theory]
        [InlineData("../../../DataFiles/workingTetMesh155.mphtxt", 1, 1, 6.94e-6, 0.0083)]
        public void SolveEquation1(string fileName, double cox, double T, double k1, double k2)
        {
            double capacity = 1;
            double dependentProductionCoeff = ((double)1 / 3) * k1 * cox * T / (k2 + cox);
            double convectionCoeff = 0;
            double independentSource = 0;
            double diffusion = 0;
            var modelProvider = new Comsol3DConvectionDiffusionProductionModelProvider(new double[] { convectionCoeff, convectionCoeff, convectionCoeff },
                diffusion, dependentProductionCoeff, independentSource, capacity);

            var model = modelProvider.CreateModelFromComsolFile(fileName);
            var solverFactory = new DenseMatrixSolver.Factory(); //Dense Matrix Solver solves with zero matrices!
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemConvectionDiffusion(model, algebraicModel, solver);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);

            var dynamicAnalyzerBuilder = new BDFDynamicAnalyzer.Builder( algebraicModel, problem, linearAnalyzer, timeStep: 1, totalTime: 10, bdfOrder: 5);
            var dynamicAnalyzer = dynamicAnalyzerBuilder.Build();

            var watchDofs = new List<(INode node, IDofType dof)>()
            {
                (model.NodesDictionary[36], ConvectionDiffusionDof.UnknownVariable),
            };

            linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            dynamicAnalyzer.Initialize();
            dynamicAnalyzer.Solve();

            DOFSLog log = (DOFSLog)linearAnalyzer.Logs[0];
            double computedValue = log.DOFValues[watchDofs[0].node, watchDofs[0].dof];

        }

        [Theory]
        [InlineData("../../../DataFiles/workingTetMesh155.mphtxt", 1, 6.94e-6, 0.0083)]
        public void SolveEquation2(string fileName, double cox, double k1, double k2)
        {
            double capacity = 1;
            double dependentProductionCoeff = k1 * cox / (k2 + cox);
            double convectionCoeff = 0;
            double independentSource = 0;
            double diffusion = 0;
            var modelProvider = new Comsol3DConvectionDiffusionProductionModelProvider(new double[] { convectionCoeff, convectionCoeff, convectionCoeff },
                diffusion, dependentProductionCoeff, independentSource, capacity);

            var model = modelProvider.CreateModelFromComsolFile(fileName);
            var solverFactory = new DenseMatrixSolver.Factory(); //Dense Matrix Solver solves with zero matrices!
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemConvectionDiffusion(model, algebraicModel, solver);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);

            var dynamicAnalyzerBuilder = new BDFDynamicAnalyzer.Builder( algebraicModel, problem, linearAnalyzer, timeStep: 1, totalTime: 10, bdfOrder: 5);
            var dynamicAnalyzer = dynamicAnalyzerBuilder.Build();

            var watchDofs = new List<(INode node, IDofType dof)>()
            {
                (model.NodesDictionary[13], ConvectionDiffusionDof.UnknownVariable),
            };

            linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            dynamicAnalyzer.Initialize();
            dynamicAnalyzer.Solve();

        }

        [Theory]
        [InlineData("../../../DataFiles/workingTetMesh155.mphtxt", 1, 1, 6.94e-6, 0.0083, new[] { 1d, 1d, 1d })]
        public void SolveEquation9(string fileName, double T, double cox, double k1, double k2, double[] vs)
        {
            double capacity = 1;
            double dependentProductionCoeff = 0;
            double independentSource = T * k1 * cox / (k2 + cox);
            double diffusion = 0;
            var modelProvider = new Comsol3DConvectionDiffusionProductionModelProviderEquation9BCs(vs,
                diffusion, dependentProductionCoeff, independentSource, capacity);

            var model = modelProvider.CreateModelFromComsolFile(fileName);
            var solverFactory = new DenseMatrixSolver.Factory(); //Dense Matrix Solver solves with zero matrices!
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemConvectionDiffusion(model, algebraicModel, solver);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);

            var dynamicAnalyzerBuilder = new BDFDynamicAnalyzer.Builder(algebraicModel, problem, linearAnalyzer, timeStep: 1, totalTime: 10, bdfOrder: 5);
            var dynamicAnalyzer = dynamicAnalyzerBuilder.Build();

            var watchDofs = new List<(INode node, IDofType dof)>()
            {
                (model.NodesDictionary[36], ConvectionDiffusionDof.UnknownVariable),
            };

            linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            dynamicAnalyzer.Initialize();
            dynamicAnalyzer.Solve();

            DOFSLog log = (DOFSLog)linearAnalyzer.Logs[0];
            double computedValue = log.DOFValues[watchDofs[0].node, watchDofs[0].dof];

        }

    }
}
