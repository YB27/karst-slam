#ifndef DIRECTED_TREE_H
#define DIRECTED_TREE_H

#include "karst_slam/typedefs.h"
#include <mrpt/graphs/dijkstra.h>
#include <mrpt/graphs/CDirectedTree.h>

namespace karst_slam{namespace graph{

// Visitor to compute the relative pose pdf between two nodes
using baseVisitor = mrpt::graphs::CDijkstra<graph_mrpt_t,typename graph_mrpt_t::maps_implementation_t>::tree_graph_t::Visitor;
struct VisitorComputeRelativePosePdf : public baseVisitor
{
    karst_slam::graph_mrpt_t * m_g; // The original graph
    mrpt::utils::TNodeID to_id;
    std::map<mrpt::utils::TNodeID,pose_pdf_t> nodes_pose_pdf;
    bool found = false;
    pose_pdf_t relativePosePdf;

    VisitorComputeRelativePosePdf(graph_mrpt_t *g,
                                  mrpt::utils::TNodeID from,
                                  mrpt::utils::TNodeID to) : m_g(g), to_id(to)
    {
        nodes_pose_pdf[from].mean  = mrpt::poses::CPose3D();//m_g->nodes[from];
        nodes_pose_pdf[from].cov_inv.unit();
        nodes_pose_pdf[from].cov_inv *= 1e6;
    }
    virtual void OnVisitNode(const mrpt::utils::TNodeID parent_id,
                             const typename baseVisitor::tree_t::TEdgeInfo &edge_to_child,
                             const size_t depth_level) MRPT_OVERRIDE
    {
        MRPT_UNUSED_PARAM(depth_level);
        const mrpt::utils::TNodeID  child_id = edge_to_child.id;

        // Compute the pose of "child_id" as parent_pose (+) edge_delta_pose,
        //  taking into account that that edge may be in reverse order
        //  and then have to invert the delta_pose:
        std::cout << "parent id : " << parent_id << std::endl;
        std::cout << "id : " << child_id << ",  edge_to_child data : " << std::endl;
        std::cout << *(edge_to_child.data) << std::endl;
        if ((!edge_to_child.reverse && !m_g->edges_store_inverse_poses) ||
                (edge_to_child.reverse &&  m_g->edges_store_inverse_poses)
                )
        {	// pose_child = p_parent (+) p_delta
            //m_g->nodes[child_id].composeFrom(m_g->nodes[parent_id],  edge_to_child.data->getPoseMean());
            nodes_pose_pdf[child_id] = nodes_pose_pdf[parent_id] + *(edge_to_child.data);
            std::cout << "nodes_pose_pdf[child_id] : " << nodes_pose_pdf[child_id] << std::endl;
        }
        else
        {	// pose_child = p_parent (+) [(-)p_delta]
            nodes_pose_pdf[child_id] = nodes_pose_pdf[parent_id] - *(edge_to_child.data);
            std::cout << "(reverse)nodes_pose_pdf[child_id] : " << nodes_pose_pdf[child_id] << std::endl;
        }
        if(child_id == to_id)
        {
            relativePosePdf = nodes_pose_pdf[child_id];
            found = true;
        }
    }
};

template<class TYPE_GRAPH>
class DirectedTree : public mrpt::graphs::CDirectedTree<const typename TYPE_GRAPH::edge_t*>
{
public:
    using base_t = mrpt::graphs::CDirectedTree<const typename TYPE_GRAPH::edge_t*>;

    void getRelativePosePdf_df(const mrpt::utils::TNodeID root,
                               VisitorComputeRelativePosePdf & user_visitor,
                               const size_t root_depth_level =0 ) const
    {
        const size_t next_depth_level = root_depth_level+1;
        typename base_t::TMapNode2ListEdges::const_iterator itChildren = this->edges_to_children.find(root);
        if (itChildren==this->edges_to_children.end()) return; // No children
        const typename base_t::TListEdges &children = itChildren->second;
        for (typename base_t::TListEdges::const_iterator itEdge=children.begin();itEdge!=children.end();++itEdge)
        {
            user_visitor.OnVisitNode(root,*itEdge,next_depth_level);
            if(user_visitor.found)
                return;
            getRelativePosePdf_df(itEdge->id,user_visitor, next_depth_level); // Recursive depth-first call.
        }
    }
    void getRelativePosePdf_bf(const mrpt::utils::TNodeID root,
                               VisitorComputeRelativePosePdf &user_visitor,
                               const size_t root_depth_level=0) const
    {
        const size_t next_depth_level = root_depth_level+1;
        typename base_t::TMapNode2ListEdges::const_iterator itChildren = this->edges_to_children.find(root);
        if (itChildren==this->edges_to_children.end()) return; // No children
        const typename base_t::TListEdges &children = itChildren->second;
        for (typename base_t::TListEdges::const_iterator itEdge=children.begin();itEdge!=children.end();++itEdge)
        {
            user_visitor.OnVisitNode(root,*itEdge,next_depth_level);
            if(user_visitor.found)
                return;
        }
        for (typename base_t::TListEdges::const_iterator itEdge=children.begin();itEdge!=children.end();++itEdge)
            getRelativePosePdf_df(itEdge->id,user_visitor,next_depth_level); // Recursive breath-first call.
    }
};
}}
#endif // DIRECTED_TREE_H
