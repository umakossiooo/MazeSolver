import Pkg; Pkg.add("FileIO")
import Pkg; Pkg.add("ImageMagick")

using Agents, Agents.Pathfinding
using FileIO # To load images you also need ImageMagick available to your project
using ImageMagick
@agent struct Walker(GridAgent{2}) end

function initialize_model(maze_map)
    # Load the maze from the image file. White values can be identified by a
    # non-zero red component
    maze = BitArray(map(x -> x.r > 0, maze_map))
    # The size of the space is the size of the maze
    space = GridSpace(size(maze); periodic = false)
    # Create a pathfinder using the AStar algorithm by providing the space and specifying
    # the walkmap parameter for the pathfinder.
    # Since we are interested in the most direct path to the end, the default
    # DirectDistance is appropriate.
    # diagonal_movement is set to false to prevent cutting corners by going along
    # diagonals.
    pathfinder = AStar(space; walkmap=maze, diagonal_movement=false)
    model = StandardABM(Walker, space; agent_step!)
    # Place a walker at the start of the maze
    add_agent!((2, 2), model)
    # The walker's movement target is the end of the maze.
    plan_route!(model[1], (1, 4), pathfinder)

    return model, pathfinder
end

agent_step!(agent, model) = move_along_route!(agent, model, pathfinder)

# Our sample walkmap
map_url =
    "https://raw.githubusercontent.com/JuliaDynamics/" *
    "JuliaDynamics/master/videos/agents/maze.bmp"
maze_map = load(download(map_url));
model, pathfinder = initialize_model(maze_map)

using CairoMakie

abmvideo(
    "maze.mp4",
    model;
    figurekwargs = (size =(900,900),),
    frames=800,
    framerate=10,
    agent_color=:blue,
    agent_size=15,
    heatarray = _ -> pathfinder.walkmap,
    add_colorbar = false,
)