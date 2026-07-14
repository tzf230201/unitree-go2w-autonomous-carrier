import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import Shutdown
from launch.events.process import ProcessExited
from launch_ros.actions import Node


def _load_controller_launch_module():
    path = Path(__file__).parents[1] / "launch" / "controller.launch.py"
    spec = importlib.util.spec_from_file_location("om6dof_controller_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _process_exit(action):
    return ProcessExited(
        returncode=1,
        action=action,
        name="controller",
        cmd=[],
        cwd=None,
        env=None,
        pid=1,
    )


def test_controller_exit_requests_full_launch_shutdown():
    launch_description = _load_controller_launch_module().generate_launch_description()
    controller = next(
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    )
    registration = next(
        entity for entity in launch_description.entities
        if isinstance(entity, RegisterEventHandler)
    )

    event = _process_exit(controller)
    assert registration.event_handler.matches(event)
    actions = registration.event_handler.handle(event, LaunchContext())

    assert len(actions) == 1
    assert isinstance(actions[0], EmitEvent)
    assert isinstance(actions[0].event, Shutdown)
    assert actions[0].event.reason == "om6dof_controller exited"


def test_controller_exit_handler_ignores_other_processes():
    launch_description = _load_controller_launch_module().generate_launch_description()
    registration = next(
        entity for entity in launch_description.entities
        if isinstance(entity, RegisterEventHandler)
    )
    other = Node(package="other_package", executable="other_node")

    assert not registration.event_handler.matches(_process_exit(other))
