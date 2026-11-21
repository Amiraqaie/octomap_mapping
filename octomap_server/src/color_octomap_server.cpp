#include "octomap_server/octomap_server.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace octomap_server
{
class ColorOctomapServer : public OctomapServer
{
public:
  using OctomapServer::OctomapServer;
};
}  // namespace octomap_server

RCLCPP_COMPONENTS_REGISTER_NODE(octomap_server::ColorOctomapServer)
