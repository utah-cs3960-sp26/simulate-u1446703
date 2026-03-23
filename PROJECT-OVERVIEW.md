Implement a 2D physics simulator. The simulator should simulate circular balls and fixed, immovable walls. The balls should be influenced by gravity and should collide with the walls and with other balls in non-elastic collisions with restitution.

Use SDL3 via C++. Running your application should set up an initial "scene" with about a thousand balls in some kind of container made up of wall pieces. Set up the initial scene so the balls bounce around for a while and then settle down. Ideally it'll also be fast enough that it's pleasant to look at.

The hard part is going to be making sure the balls don't end up overlapping or squeezing through the walls. You'll also sometimes see balls start vibrating really fast. Sometimes they vibrate fast and faster and eventually shoot off to infinity. Make the restitution amount configurable; you should see things settle down faster with less restitution, but the final "settled" state should take up the same amount of space no matter the amount of restitution.

Make it possible to describe the initial scene in a CSV file; the CSV file should have one row per ball and list a starting position and a color. The CSV should also be capable of describing walls in a similar manor. Make the simulator save the final positions to a similar CSV file. Add a tool that takes an initial scene CSV file and assign colors based on where the final balls end up and what color a given image has at that location.

# Implementation notes:
- The project should be able to comfortably render 1000 balls at 30 FPS most of the time.