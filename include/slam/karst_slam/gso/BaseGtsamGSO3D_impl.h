#ifndef BASEGTSAMGSO3D_IMPL_H
#define BASEGTSAMGSO3D_IMPL_H

template <class DERIVED>
void BaseGtsamGSO3D<DERIVED>::execOptimization(const std::set<mrpt::utils::TNodeID>* nodes_to_optimize)
{
    using namespace mrpt_gtsam::wrapperGTSAM;
    using namespace karst_slam::graph;
    using gtsam_t = PoseGraph::graph_converter_t::gtsam_value_t;

    // Update the gtsam converted graph
    this->m_pose_graph->updateConvertedGraph();
    //m_graphConverter.updateConvertedGraph();

    // Get the subgraph corresponding to the nodes_to_optimize (or all if NULL)
    gtsamGraph gtsam_graph = this->m_pose_graph->getConvertedGTSAMGraph();
    if(nodes_to_optimize != nullptr)
        gtsam_graph = gtsam_graph.extractSubgraph<gtsam_t>(nodes_to_optimize, true);

//    gtsamGraph gtsam_graph ;
//    if(nodes_to_optimize == nullptr)
//        gtsam_graph = m_pose_graph->getConvertedGTSAMGraph();//m_graphConverter.getConvertedGTSAMGraph();
//    else
//        gtsam_graph = m_pose_graph->//this->m_graphConverter.getConvertedGTSAMGraph().template extractSubgraph<gtsam_t>(nodes_to_optimize, true);

    // Run the wrapped optimizer
    gtsam::Values optimizedValues = execOptimization_(gtsam_graph, nodes_to_optimize);

    // Set back the new values to the mrpt and converted graph
    this->m_pose_graph->updateAfterOptimization(optimizedValues);//this->m_graphConverter.updateAfterOptimization(optimizedValues);

    // Update the marginals (ie the covariance matrix of each node)
    // now should be working (before problems with how the 3D euler poses were represented in GTSAM)
    this->m_pose_graph->updateMarginals();
}

template <class DERIVED>
void BaseGtsamGSO3D<DERIVED>::loadOptimizerParams(const mrpt::utils::CConfigFile& source)
{
    loadOptimizerParams_(source);
    printParams_();
}

template <class DERIVED>
void BaseGtsamGSO3D<DERIVED>::loadNonLinearParams(const mrpt::utils::CConfigFile& source)
{
    convertToNonLinearOptimizerParams(source,"OptimizerParameters", m_params);
}

template <class DERIVED>
void BaseGtsamGSO3D<DERIVED>::printNonLinearParams() const
{
    using namespace std;
    using namespace mrpt_gtsam::wrapperGTSAM;

    cout << "------- [CBaseGtsamGSO Parameters] ---------" << endl;
    cout << "maxIterations       = "  << this->m_params.maxIterations    << endl;
    cout << "relativeErrorTol    = "  << this->m_params.relativeErrorTol << endl;
    cout << "absoluteErrorTol    = "  << this->m_params.absoluteErrorTol << endl;
    cout << "errorTol            = "  << this->m_params.errorTol         << endl;
    cout << "verbosity           = "  << this->m_params.verbosity        << endl;
    cout << "orderingType        = "  << gtsamOrderingTypeValue2Name.at(this->m_params.orderingType)     << endl;
    cout << "linearSolverType    = "  << gtsamNLLinearSolverTypeValue2Name.at(this->m_params.linearSolverType) << endl;
    cout << endl;
}

#endif //BASEGTSAMGSO3D_IMPL_H
