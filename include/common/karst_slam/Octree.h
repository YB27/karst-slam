// Code taken and adapted from https://github.com/kipr/
#include <vector>
#include <mrpt/math/lightweight_geom_data.h>

namespace karst_slam{
/**
 * Class defining an Octree to efficiently store and retrieve 3d data
 * Code taken and adapted from https://github.com/kipr/
*/
class Octree
{
public:
    /** Structure defining a Node in the Octree */
    struct Node
    {   
        /** Default ctor */
        Node() {}
        int begin, end;  
        float x_min, x_max, y_min, y_max, z_min, z_max; //!< node spatial bounds
        int maxLevels; 
        bool isLeaf; //!< True if leaf of the tree (no child)
        int children[8]; //!< 8 children nodes index 
    };

    Octree();
    Octree(const std::vector<mrpt::math::TPoint3D>& points, int maxLevels = 10, int minPoints = 20);
    virtual ~Octree();

    virtual void buildTree(const std::vector<mrpt::math::TPoint3D>& points, int maxLevels = 10, int minPoints = 20);
    virtual void getPointsWithinSphere(const mrpt::math::TPoint3D& center, float radius,
        std::vector<mrpt::math::TPoint3D>& points) const;
    const std::vector<Node>& getNodes() const { return nodes; }
private:
    int minPoints;
    std::vector<mrpt::math::TPoint3D> points;
    std::vector<Node> nodes;

    virtual void buildNext(size_t node_ind);
};
}