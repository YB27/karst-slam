#include "karst_slam/Octree.h"
#include <limits>
#include <cassert>

using namespace std;
using namespace karst_slam;
using namespace mrpt::math;

const size_t MAX_STACK_SIZE = 255;
const size_t MAX_LEAFS = 8;

bool checkIfNodeOutsideSphere(const Octree::Node& node, const TPoint3D& c, float r)
{
    if (node.x_max < (c.x - r) || node.y_max < (c.y - r) || node.z_max < (c.z - r))
        return true;

    if ((c.x + r) < node.x_min || (c.y + r) < node.y_min || (c.z + r) < node.z_min)
        return true;

    return false;
}

bool checkIfNodeInsideSphere(const Octree::Node& node, const TPoint3D& c, float r)
{
    r *= r;

    float d2_xmin = (node.x_min - c.x) * (node.x_min - c.x);
    float d2_ymin = (node.y_min - c.y) * (node.y_min - c.y);
    float d2_zmin = (node.z_min - c.z) * (node.z_min - c.z);

    if (d2_xmin + d2_ymin + d2_zmin > r)
        return false;

    float d2_zmax = (node.z_max - c.z) * (node.z_max - c.z);

    if (d2_xmin + d2_ymin + d2_zmax > r)
        return false;

    float d2_ymax = (node.y_max - c.y) * (node.y_max - c.y);

    if (d2_xmin + d2_ymax + d2_zmin > r)
        return false;

    if (d2_xmin + d2_ymax + d2_zmax > r)
        return false;

    float d2_xmax = (node.x_max - c.x) * (node.x_max - c.x);

    if (d2_xmax + d2_ymin + d2_zmin > r)
        return false;

    if (d2_xmax + d2_ymin + d2_zmax > r)
        return false;

    if (d2_xmax + d2_ymax + d2_zmin > r)
        return false;

    if (d2_xmax + d2_ymax + d2_zmax > r)
        return false;

    return true;
}

void fillMinMax(const vector<TPoint3D>& points, Octree::Node& node)
{
    node.x_max = node.y_max = node.z_max = numeric_limits<float>::min();
    node.x_min = node.y_min = node.z_min = numeric_limits<float>::max();

    for (size_t i = 0; i < points.size(); ++i)
    {
        const TPoint3D& point = points[i];

        if (node.x_max < point.x)
            node.x_max = point.x;

        if (node.y_max < point.y)
            node.y_max = point.y;

        if (node.z_max < point.z)
            node.z_max = point.z;

        if (node.x_min > point.x)
            node.x_min = point.x;

        if (node.y_min > point.y)
            node.y_min = point.y;

        if (node.z_min > point.z)
            node.z_min = point.z;
    }
}

size_t findSubboxForPoint(const TPoint3D& point, const Octree::Node& node)
{
    size_t ind_x = point.x < (node.x_max + node.x_min) / 2 ? 0 : 1;
    size_t ind_y = point.y < (node.y_max + node.y_min) / 2 ? 0 : 1;
    size_t ind_z = point.z < (node.z_max + node.z_min) / 2 ? 0 : 1;

    return (ind_x << 2) + (ind_y << 1) + (ind_z << 0);
}
void initChildBox(const Octree::Node& parent, size_t boxIndex, Octree::Node& child)
{
    child.x_min = child.x_max = (parent.x_max + parent.x_min) / 2;
    child.y_min = child.y_max = (parent.y_max + parent.y_min) / 2;
    child.z_min = child.z_max = (parent.z_max + parent.z_min) / 2;

    if ((boxIndex >> 0) & 1)
        child.z_max = parent.z_max;
    else
        child.z_min = parent.z_min;

    if ((boxIndex >> 1) & 1)
        child.y_max = parent.y_max;
    else
        child.y_min = parent.y_min;

    if ((boxIndex >> 2) & 1)
        child.x_max = parent.x_max;
    else
        child.x_min = parent.x_min;
}

////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////       Octree       //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
Octree::Octree()
{
}

Octree::Octree(const vector<TPoint3D>& points3d, int maxLevels, int _minPoints)
{
    buildTree(points3d, maxLevels, _minPoints);
}

Octree::~Octree()
{
}

void Octree::getPointsWithinSphere(const TPoint3D& center, float radius, vector<TPoint3D>& out) const
{
    out.clear();

    if (nodes.empty())
        return;

    int stack[MAX_STACK_SIZE];
    int pos = 0;
    stack[pos] = 0;

    while (pos >= 0)
    {
        const Node& cur = nodes[stack[pos--]];

        if (checkIfNodeOutsideSphere(cur, center, radius))
            continue;

        if (checkIfNodeInsideSphere(cur, center, radius))
        {
            size_t sz = out.size();
            out.resize(sz + cur.end - cur.begin);
            for (int i = cur.begin; i < cur.end; ++i)
                out[sz++] = points[i];
            continue;
        }

        if (cur.isLeaf)
        {
            double r2 = radius * radius;
            size_t sz = out.size();
            out.resize(sz + (cur.end - cur.begin));

            for (int i = cur.begin; i < cur.end; ++i)
            {
                const TPoint3D& point = points[i];

                double dx = (point.x - center.x);
                double dy = (point.y - center.y);
                double dz = (point.z - center.z);

                double dist2 = dx * dx + dy * dy + dz * dz;

                if (dist2 < r2)
                    out[sz++] = point;
            };
            out.resize(sz);
            continue;
        }

        if (cur.children[0])
            stack[++pos] = cur.children[0];

        if (cur.children[1])
            stack[++pos] = cur.children[1];

        if (cur.children[2])
            stack[++pos] = cur.children[2];

        if (cur.children[3])
            stack[++pos] = cur.children[3];

        if (cur.children[4])
            stack[++pos] = cur.children[4];

        if (cur.children[5])
            stack[++pos] = cur.children[5];

        if (cur.children[6])
            stack[++pos] = cur.children[6];

        if (cur.children[7])
            stack[++pos] = cur.children[7];
    }
}

void Octree::buildTree(const vector<TPoint3D>& points3d, int maxLevels, int _minPoints)
{
    assert((size_t)maxLevels * 8 < MAX_STACK_SIZE);
    points.resize(points3d.size());
    std::copy(points3d.begin(), points3d.end(), points.begin());
    minPoints = _minPoints;

    nodes.clear();
    nodes.push_back(Node());
    Node& root = nodes[0];
    fillMinMax(points, root);

    root.isLeaf = true;
    root.maxLevels = maxLevels;
    root.begin = 0;
    root.end = (int)points.size();
    for (size_t i = 0; i < MAX_LEAFS; i++)
        root.children[i] = 0;

    if (maxLevels != 1 && (root.end - root.begin) > _minPoints)
    {
        root.isLeaf = false;
        buildNext(0);
    }
}

void  Octree::buildNext(size_t nodeInd)
{
    size_t size = nodes[nodeInd].end - nodes[nodeInd].begin;

    vector<size_t> boxBorders(MAX_LEAFS + 1, 0);
    vector<size_t> boxIndices(size);
    vector<TPoint3D> tempPoints(size);

    for (int i = nodes[nodeInd].begin, j = 0; i < nodes[nodeInd].end; ++i, ++j)
    {
        const TPoint3D& p = points[i];

        size_t subboxInd = findSubboxForPoint(p, nodes[nodeInd]);

        boxBorders[subboxInd + 1]++;
        boxIndices[j] = subboxInd;
        tempPoints[j] = p;
    }

    for (size_t i = 1; i < boxBorders.size(); ++i)
        boxBorders[i] += boxBorders[i - 1];

    vector<size_t> writeInds(boxBorders.begin(), boxBorders.end());

    for (size_t i = 0; i < size; ++i)
    {
        size_t boxIndex = boxIndices[i];
        TPoint3D& curPoint = tempPoints[i];

        size_t copyTo = nodes[nodeInd].begin + writeInds[boxIndex]++;
        points[copyTo] = curPoint;
    }

    for (size_t i = 0; i < MAX_LEAFS; ++i)
    {
        if (boxBorders[i] == boxBorders[i + 1])
            continue;

        nodes.push_back(Node());
        Node& child = nodes.back();
        initChildBox(nodes[nodeInd], i, child);

        child.isLeaf = true;
        child.maxLevels = nodes[nodeInd].maxLevels - 1;
        child.begin = nodes[nodeInd].begin + (int)boxBorders[i + 0];
        child.end = nodes[nodeInd].begin + (int)boxBorders[i + 1];
        for (size_t k = 0; k < MAX_LEAFS; k++)
            child.children[k] = 0;

        nodes[nodeInd].children[i] = (int)(nodes.size() - 1);

        if (child.maxLevels != 1 && (child.end - child.begin) > minPoints)
        {
            child.isLeaf = false;
            buildNext(nodes.size() - 1);
        }
    }
}
