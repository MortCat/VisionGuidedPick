#pragma once
#include <filesystem>
#include "ptc_core/config/system_settings.hpp"
#include "ptc_app/endpoint_config.hpp"

namespace ptc_app
{
struct ResourceIdentity
{std::string kind, identity, sha256; std::filesystem::path path;};
struct SettingsLoadResult
{
  std::optional<ptc_core::SettingsSnapshot> snapshot;
  std::vector<ptc_core::ConfigError> errors;
  std::vector<ResourceIdentity> resources;
  EndpointConfig endpoints;
  std::string binding_revision;
};
class SettingsLoader
{
public:
  SettingsLoadResult load(const std::filesystem::path & yaml_path) const;
};
// Shared by resource verification and tests. Throws on I/O or hashing failure.
std::string sha256_file(const std::filesystem::path &);
std::string sha256_text(const std::string &);
}  // namespace ptc_app
