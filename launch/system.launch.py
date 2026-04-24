"""Direct ROS 2 launch entry point for the canonical III system graph."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from iii_drone_supervision.system_spec import build_system_launch_description


def _build_launch(context, *args, **kwargs):
    del args, kwargs
    profile = LaunchConfiguration("profile").perform(context)
    return list(build_system_launch_description(profile).entities)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="sim",
                description="System profile to launch (sim, real, opti_track).",
            ),
            OpaqueFunction(function=_build_launch),
        ]
    )
