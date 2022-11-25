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
	public class EquationsTests1
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

		public EquationsTests1()
		{
            IsoparametricJacobian3D.DeterminantTolerance = 1e-20;
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
            var modelProvider = new GenericComsol3DConvectionDiffusionProductionModelProvider(new double[] { convectionCoeff, convectionCoeff, convectionCoeff },
                diffusion, dependentProductionCoeff, independentSource, capacity);

            var model = modelProvider.CreateModelFromComsolFile(fileName);
            modelProvider.AddTopAndBottomBCs(model, 2, 1, 0, 1);
            modelProvider.AddInitialConditionsForTheRestOfBulkNodes(model, 2, 0, 0);


            var solverFactory = new DenseMatrixSolver.Factory() { IsMatrixPositiveDefinite = false}; //Dense Matrix Solver solves with zero matrices!
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemConvectionDiffusion(model, algebraicModel, solver);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);

            var dynamicAnalyzerBuilder = new NewmarkDynamicAnalyzer.Builder( algebraicModel, problem, linearAnalyzer, timeStep: 1, totalTime: 10);
            var dynamicAnalyzer = dynamicAnalyzerBuilder.Build();

            var watchDofs = new List<(INode node, IDofType dof)>()
            {
                (model.NodesDictionary[36], ConvectionDiffusionDof.UnknownVariable),
            };

            linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            dynamicAnalyzer.ResultStorage = new ImplicitIntegrationAnalyzerLog();
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
            var solverFactory = new DenseMatrixSolver.Factory() { IsMatrixPositiveDefinite = true}; //Dense Matrix Solver solves with zero matrices!
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

        /// <summary>
        /// 
        /// </summary>
        /// <param name="fileName"></param>
        /// <param name="kth"> gia to normal part 7.52×10-13 m2/kPasec  </param>
        /// <param name="Lp"> gia to normal part 2.7x10-9 m/kPa?sec </param>
        /// <param name="Sv">    7×103 m-1 for normal tissue</param>
        /// <param name="pv"> 4kPa </param>
        /// <param name="LplSvl">3.75 x10-1 [1/(kPa?sec)]</param>
        /// <param name="pl">0 Kpa</param>
        /// <param name="div_vs">scalar. In next development step it will be distributed in space from structural solution</param>
        [Theory]
        //[InlineData("../../../DataFiles/workingTetMesh155.mphtxt", 7.52e-13, /*2.7e-9*/1, 7e3, /*4*/1,
        //                                                          /*3.75e-1*/ 1, 0, 0)]
        [InlineData("../../../DataFiles/3d8Hexa.mphtxt", 1, /*2.7e-9*/1, -1, /*4*/-1,
                                                                  /*3.75e-1*/ 0, 0, 0)]
        public void SolveEquation7and8ofUpdatedReportStaticZero(string fileName, double kth, double Lp, double Sv, double pv,
                                                                double LplSvl, double pl, double div_vs)
        {
            double convectionCoeff = 1;

            double capacity = 0;
            double dependentProductionCoeff = -(Lp * Sv + LplSvl);
            double independentSource = Lp * Sv * pv + LplSvl * pl - div_vs;
            double diffusion = kth;

            var modelProvider = new GenericComsol3DConvectionDiffusionProductionModelProvider(new double[] { convectionCoeff, convectionCoeff, convectionCoeff },
                diffusion, dependentProductionCoeff, independentSource, capacity);

            var model = modelProvider.CreateModelFromComsolFile(fileName);
            modelProvider.AddTopAndBottomBCs(model, 2, 100, 0, 50);
            var solverFactory = new DenseMatrixSolver.Factory() { IsMatrixPositiveDefinite = true}; //Dense Matrix Solver solves with zero matrices!
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemConvectionDiffusion(model, algebraicModel, solver);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);

            //var dynamicAnalyzerBuilder = new StaticAnalyzer.Builder(model, algebraicModel, problem, linearAnalyzer, timeStep: 1, totalTime: 10, bdfOrder: 5);
            //var cAnalyzer = dynamicAnalyzerBuilder.Build();
            var staticAnalyzer = new StaticAnalyzer( algebraicModel, problem, linearAnalyzer);


            //dynamicAnalyzer.ResultStorage = new ImplicitIntegrationAnalyzerLog();

            //var watchDofs = new List<(INode node, IDofType dof)>()
            //{
            //    (model.NodesDictionary[36], ConvectionDiffusionDof.UnknownVariable),
            //};

            //linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            staticAnalyzer.Initialize();
            staticAnalyzer.Solve();

            DOFSLog log = (DOFSLog)linearAnalyzer.Logs[0];
            double computedValue = log.DOFValues[watchDofs[0].node, watchDofs[0].dof];

        }

    }
}
