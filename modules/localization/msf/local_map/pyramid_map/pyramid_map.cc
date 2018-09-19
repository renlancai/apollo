#include "modules/localization/msf/local_map/pyramid_map/pyramid_map.h"
#include "modules/localization/msf/local_map/pyramid_map/pyramid_map_node.h"

namespace apollo {
namespace localization {
namespace msf {

PyramidMap::PyramidMap(PyramidMapConfig* config) : BaseMap(config) {
}

PyramidMap::~PyramidMap() {
}

float PyramidMap::GetIntensitySafe(
            const Eigen::Vector3d& coordinate, int zone_id, 
            unsigned int resolution_id, unsigned int level) {
    MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(
        GetMapConfig(), coordinate, resolution_id, zone_id);
    PyramidMapNode* node =
        dynamic_cast<PyramidMapNode*>(GetMapNodeSafe(index));
    return node->GetIntensitySafe(coordinate, level);        
}

float PyramidMap::GetIntensityVarSafe(
            const Eigen::Vector3d& coordinate, int zone_id, 
            unsigned int resolution_id, unsigned int level) {
    MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(
        GetMapConfig(), coordinate, resolution_id, zone_id);
    PyramidMapNode* node =
        dynamic_cast<PyramidMapNode*>(GetMapNodeSafe(index));
    return node->GetIntensityVarSafe(coordinate, level);
}

float PyramidMap::GetAltitudeSafe(
            const Eigen::Vector3d& coordinate, int zone_id, 
            unsigned int resolution_id, unsigned int level) {
    MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(
        GetMapConfig(), coordinate, resolution_id, zone_id);
    PyramidMapNode* node =
        dynamic_cast<PyramidMapNode*>(GetMapNodeSafe(index));
    return node->GetAltitudeSafe(coordinate, level);
}

float PyramidMap::GetAltitudeVarSafe(
            const Eigen::Vector3d& coordinate, int zone_id, 
            unsigned int resolution_id, unsigned int level) {
    MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(
        GetMapConfig(), coordinate, resolution_id, zone_id);
    PyramidMapNode* node =
        dynamic_cast<PyramidMapNode*>(GetMapNodeSafe(index));
    return node->GetAltitudeVarSafe(coordinate, level);
}

float PyramidMap::GetGroundAltitudeSafe(
            const Eigen::Vector3d& coordinate, int zone_id, 
            unsigned int resolution_id, unsigned int level) {
    MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(
        GetMapConfig(), coordinate, resolution_id, zone_id);
    PyramidMapNode* node =
        dynamic_cast<PyramidMapNode*>(GetMapNodeSafe(index));
    return node->GetGroundAltitudeSafe(coordinate, level);
}

unsigned int PyramidMap::GetCountSafe(
            const Eigen::Vector3d& coordinate, int zone_id, 
            unsigned int resolution_id, unsigned int level) {
    MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(
        GetMapConfig(), coordinate, resolution_id, zone_id);
    PyramidMapNode* node =
        dynamic_cast<PyramidMapNode*>(GetMapNodeSafe(index));
    return node->GetCountSafe(coordinate, level);  
}

unsigned int PyramidMap::GetGroundCountSafe(
            const Eigen::Vector3d& coordinate, int zone_id, 
            unsigned int resolution_id, unsigned int level) {
    MapNodeIndex index = MapNodeIndex::GetMapNodeIndex(
        GetMapConfig(), coordinate, resolution_id, zone_id);
    PyramidMapNode* node =
        dynamic_cast<PyramidMapNode*>(GetMapNodeSafe(index));
    return node->GetGroundCountSafe(coordinate, level);
}

} // namespace msf
} // namespace localization
} // namespace apollo