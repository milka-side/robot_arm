alias cb="colcon build --symlink-install"
alias cba="colcon build --symlink-install --packages-select-regex "^arm_""
alias sws="if [ -f install/setup.bash ]; then source install/setup.bash && echo 'Workspace sourced!'; else echo 'No install/setup.bash found in this directory.'; fi"
alias sb="if [ -f install/setup.bash ]; then source install/setup.bash && echo 'Workspace sourced!'; else echo 'No install/setup.bash found in this directory.'; fi"

alias tl="ros2 topic list"
alias nl="ros2 node list"
alias te="ros2 topic echo"

kill_node() {
    if [ -z "$1" ]; then
        echo "Usage: kill_node <node_or_executable_name>"
        echo "Example: kill_node minimal_publisher"
        return 1
    fi

    local target="$1"
    
    # 1. Try graceful termination (SIGINT - simulates Ctrl+C)
    echo "Attempting to gracefully stop '$target'..."
    pkill -SIGINT -f "$target"
    
    # Wait to allow node to clean up its DDS entities
    sleep 2
    
    # 2. Check if it's still running, and forcefully kill if necessary (SIGKILL)
    if pgrep -f "$target" > /dev/null; then
        echo "Node '$target' is still hanging. Forcing shutdown..."
        pkill -SIGKILL -f "$target"
        echo "Killed."
    else
        echo "Successfully shut down '$target'."
    fi
}