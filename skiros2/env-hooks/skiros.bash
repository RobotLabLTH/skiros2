
# If you change this file, please increment the version number in
# CMakeLists.txt to trigger a CMake update.

function skiros() {
    case $1 in
	wm)
	    rosrun skiros2_world_model ${@:2}
	    ;;
	sm)
	    rosrun skiros2_skill ${@:2}
	    ;;
	tm)
	    rosrun skiros2_task ${@:2}
	    ;;
    esac
}

function _skiros() {
    local cur="${COMP_WORDS[COMP_CWORD]}"
    local cmd="${COMP_WORDS[1]}"

    case "${COMP_CWORD}" in
	1)
	    COMPREPLY=( $(compgen -W "wm sm" -- $cur) )
	    ;;
	2)
	    local workspace=$(roscd && cd .. && pwd)

	    case "${cmd}" in
		wm|world_model)
		    local launchfiles=$(find $(rospack find skiros2_world_model)/nodes/utils -name '*' -type f -printf "%f\n")
		    COMPREPLY=( $(compgen -W "${launchfiles}" -- $cur) )
		    ;;
		sm|skill_mgr)
		    local launchfiles=$(find $(rospack find skiros2_skill)/nodes/utils -name '*' -type f -printf "%f\n")
		    COMPREPLY=( $(compgen -W "${launchfiles}" -- $cur) )
		    ;;
		tm|task_mgr)
		    local launchfiles=$(find $(rospack find skiros2_task)/nodes/utils -name '*' -type f -printf "%f\n")
		    COMPREPLY=( $(compgen -W "${launchfiles}" -- $cur) )
		    ;;
	    esac
	    ;;
	*)
	    COMPREPLY=""
	    ;;
    esac
}
complete -F _skiros skiros
