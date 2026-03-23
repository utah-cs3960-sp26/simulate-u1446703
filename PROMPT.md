# Context Files

The following files should always be pulled into context:

- ./AGENTS.md: Agent guideline file, and documentation guide
- ./PROJECT-OVERVIEW.md: More detailed information about the project specifications
- ./TASKS.md: Planning file used to keep track of what tasks are being worked on

# Prompt
*See PROJECT-OVERVIEW.md for more information when necessary*

You are implementing a 2D physics simulator using SDL3 via C++.. The simulator should simulate circular balls and fixed, immovable walls. The balls should be influenced by gravity and should collide with the walls and with other balls in non-elastic collisions with restitution. Balls must not overlap or phase through walls or other balls. Make the restitution amount configurable; you should see things settle down faster with less restitution, but the final "settled" state should take up the same amount of space no matter the amount of restitution. The project should be able to comfortably render 1000 balls at 30 FPS most of the time.

There must be accurate and adequate tests for this simulator. You must at all costs verify that the simulator is working after every change, by whatever means necessary. You are encouraged to launch the program yourself and verify that it works properly.

# Multi-agent context

You are one instance of a repeating prompt. You're job is to create, test, and refine this project to best meet the specifications outlined in ./PROJECT-OVERVIEW.md. You are to always optimize for future iterations to cohesively take over from where you leave off. This includes but is not limited too:
- Commenting your code in a way that is readable to humans and agents, and allows either to easily take over work
- Do not end a thread until the code is in a place where the next iteration can pick up where you left off
- Always test your changes, through both unit tests and running the program yourself to test it.
- Document everything you can, and always refer to relevant documentation when working
- Utilize TASKS.md to keep track of what you're currently working on, and what needs to be done. Every plan or work that needs to exist outside of one iteration must be kept track of there.
- Plan out implementations in advance, and use TASKS.md to faciliate this. You are welcome to create more markdown files if necessary.
- If at any time you determine a way to optimize the information handoff between agents, you are encouraged to do so. You are allowed to make any changes you deem necessary to any file **besides PROMPT>.md**.

**There will be no human intervention with this project**. You must account for this fact. The AI model will not necessarily be the same between iterations, and may change as the user deems fit.

# Self-improvement

You are allowed and encouraged to create more markdown files to facilitate the project design process.

# Work Documentation
Document all progress in AGENT-PROGRESS.md. Don't delete anything already there, only log your progress in it.