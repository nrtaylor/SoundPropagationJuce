# SoundPropagationJuce
Juce App for setting up my personal test bed for sound propagation ideas.

See branch "First-2D-Demo" for the last stable commit.

Current Stage:
- First frequency and time dependent modes added.
- Refactoring after adding "planners" to handle simulation instead of the rooms.
- Supporting up to 3 sources
- Make code re-useable outside of the test bed.

Next Stage:
- Thinking about beginning FEM set-up.
- Continue to refactor.

Known limitations to fix later:
- The apps screen dimenisions dictates the resolution of the generated image used to display results.
- Frequency mode does not generate audio.
- A* mode is inefficient

![alt text](/Images/SoundPropagationTestBed.png)
![alt text](/Images/SoundPropagation-Planned_Features.png)
