Here is the content for the README file:

---

# 2D CFAR Implementation in MATLAB

## Overview

This MATLAB code implements the 2D Constant False Alarm Rate (CFAR) detection process on a simulated Range-Doppler Map (RDM) obtained from an FMCW radar. The CFAR process helps in detecting the target by dynamically adjusting the detection threshold based on the noise level in the surrounding cells.

## Implementation Steps for the 2D CFAR Process

1. **Range-Doppler Map (RDM) Generation:**
   - Generate a simulated radar scenario using FMCW waveform.
   - Perform 2D FFT on the radar signal to obtain the RDM.

2. **Training and Guard Cells Selection:**
   - Choose the number of training cells (`Tr` for range and `Td` for Doppler) surrounding the Cell Under Test (CUT) to estimate the noise level.
   - Select the number of guard cells (`Gr` for range and `Gd` for Doppler) around the CUT to prevent the target signal from affecting the noise estimate.

3. **Threshold Calculation:**
   - Slide a window across the RDM, where for each position, the noise level is estimated by averaging the power in the training cells.
   - Convert the noise level from dB to power using the `db2pow` function.
   - Convert the average power back to dB using the `pow2db` function and add an offset to set the detection threshold.
   - Compare the CUT against the threshold. If the CUT value exceeds the threshold, it is marked as a target (1); otherwise, it is considered noise (0).

4. **Edge Suppression:**
   - Due to the nature of CFAR, the edges of the RDM cannot be properly thresholded since the CUT window extends beyond the map's boundaries.
   - To maintain the same size for the output map, the cells at the edges that cannot be thresholded are set to 0.

5. **Output Visualization:**
   - The CFAR output is visualized using a 3D surface plot, showing the detected targets in the Range-Doppler domain.

## Selection of Training, Guard Cells, and Offset

- **Training Cells (`Tr`, `Td`):** 7 cells are selected in both the range and Doppler dimensions. These cells provide an estimate of the noise level in the vicinity of the CUT.
- **Guard Cells (`Gr`, `Gd`):** 2 cells are selected in both the range and Doppler dimensions. These cells protect the CUT from including the target signal in the noise estimation.
- **Offset:** An offset of 5 dB is added to the noise level to set the detection threshold. This helps to balance the false alarm rate and the probability of detection.

## Steps Taken to Suppress the Non-Thresholded Cells at the Edges

- The cells at the edges of the RDM, where the sliding window extends beyond the boundaries of the matrix, are set to 0. This is done to ensure that the output map has the same dimensions as the input RDM, even though the CUT could not be processed at the edges.
