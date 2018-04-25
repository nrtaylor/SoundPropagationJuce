# SoundPropagationJuce
Juce App for setting up my personal test bed for sound propagation ideas.

Current Stage:
- Cleaning up/optimizing an A* search from receiver to source as an estimated propagation distance.

Next Stage:
- New mode that use "edges" (endpoints or concave angles) as nodes for finding multiple propagation paths.
- Refactor geometric attenuation out of Simulate and integrate the atmospheric attenuation based on the frequecny slider.

Known limitations to fix later:
- Audio files are hardcoded.
- The apps screen dimenisions dictates the resolution of the generated image used to display results.
- Receiver location is fixed at 0,0.

![alt text](/Images/SoundPropagation-A_Star-4_2018.png)
![alt text](/Images/SoundPropagation-Planned_Features-4_2018.png)
