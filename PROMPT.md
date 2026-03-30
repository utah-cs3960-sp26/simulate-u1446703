# Context Files

The following files should always be pulled into context:

- ./AGENTS.md: Agent guideline file, and documentation guide
- ./PROJECT-OVERVIEW.md: More detailed information about the project specifications
- ./TASKS.md: Planning file used to keep track of what tasks are being worked on

# Prompt
*See PROJECT-OVERVIEW.md for more information when necessary*

You are implementing a 2D physics simulator using SDL3 via C++.. The simulator should simulate circular balls and fixed, immovable walls. The balls should be influenced by gravity and should collide with the walls and with other balls in non-elastic collisions with restitution. Balls must not overlap or phase through walls or other balls. Make the restitution amount configurable; you should see things settle down faster with less restitution, but the final "settled" state should take up the same amount of space no matter the amount of restitution. The project should be able to comfortably render 1000 balls at 30 FPS most of the time.

**Every Ball should be affected by physics at all times**. This is **vitally important** and must be thoroughly checked for. If balls "freeze" in place, this needs to be

Make it possible to describe the initial scene in a CSV file; the CSV file should have one row per ball and list a starting position and a color. The CSV should also be capable of describing walls in a similar manor. Make the simulator save the final positions to a similar CSV file.

Add a tool that takes an initial scene CSV file and assign colors based on where the final balls end up and what color a given image has at that location. The end result of running this tool should produce a CSV that you can run, and have all the balls produce an image when they finish settling

There must be accurate and adequate tests for this simulator. You must at all costs verify that the simulator is working after every change, by whatever means necessary. You are encouraged to launch the program yourself and verify that it works properly.

# Priorities

You must **always** make sure existing features are up to spec before beginning on a new feature. New features should not be created, nor their tests created, until every existing and core feature works flawlessly. It may take multiple iterations to make this happen. You're goal is polish, not fancy features.

Tests should **always** have a reasonable timeout so they don't run forever. You should seperate tests that take a while (more than a few seconds) into their own file so they can only be run when you are working on a relevant part of the code base. Additionally, when possible you should only run relevant tests instead of all of them in order to save time and context

# Multi-agent context

You are one instance of a repeating prompt. You're job is to create, test, and refine this project to best meet the specifications outlined in ./PROJECT-OVERVIEW.md. You are to always optimize for future iterations to cohesively take over from where you leave off. This includes but is not limited too:
- Commenting your code in a way that is readable to humans and agents, and allows either to easily take over work
- Do not end a thread until the code is in a place where the next iteration can pick up where you left off
- Always test your changes, through both unit tests and running the program yourself to test it. Create unit tests before implementing a new feature, and build off of that.
- Every feature mentioned in PROJECT-OVERVIEW.md must be thoroughly tested
- Document everything you can, and always refer to relevant documentation when working
- Utilize TASKS.md to keep track of what you're currently working on, and what needs to be done. Every plan or work that needs to exist outside of one iteration must be kept track of there.
- Plan out implementations in advance, and use TASKS.md to faciliate this. You are welcome to create more markdown files if necessary.
- If at any time you determine a way to optimize the information handoff between agents, you are encouraged to do so. You are allowed to make any changes you deem necessary to any file **besides PROMPT>.md**.

**There will be no human intervention with this project**. You must account for this fact. The AI model will not necessarily be the same between iterations, and may change as the user deems fit.

# Self-improvement
You are allowed and encouraged to create more markdown files to facilitate the project design process.

# Work Documentation
Document all progress in AGENT-PROGRESS.md. Don't delete anything already there, only log your progress in it.

# Development Context

You are operating within a docker container. You have complete free reign to do whatever you please within this container, and do not worry about asking the user for permission.

Additionally, you do not have a GUI to work with. You must still test and develop GUI features independently. You may work around this by using the programs built in screenshot function to take screenshots at different parts throughout testing and verification.

I repeat and emphasize, despite the limitation of no GUI you **must check and verify that everything works properly regardless** even if you must come up with work arounds for such.