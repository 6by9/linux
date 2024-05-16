.. SPDX-License-Identifier: GFDL-1.1-no-invariants-or-later

.. _tof-controls:

***************************************
Time of Flight Camera Control Reference
***************************************

The Time of Flight class includes controls for digital features
of the TOF sensor.

TOF sensor is a receiver chip. Each pixel in the sensor measures the travel time
of light to that pixel and hence the distance to the object seen by that pixel.
There are different types of TOF sensors. Direct TOF sensors (also known
as Lidars) send a single pulse and measure direct time of flight.
Another type of TOF is indirect TOF sensors, which emit continuous wave
(could be radio or infrared) and then measure phase shift of reflected light.
The sensor modulates outgoing light and then collects reflected photons
as an electric charge with modulated pattern. Knowing the frequency of
the pattern you can calculate the real distance.

For more information about TOF sensors see
`TOF <https://en.wikipedia.org/wiki/Time-of-flight_camera>`__ from Wikipedia.
Also, there are other nice explanations from vendors about indirect TOF:
`Microsoft <https://devblogs.microsoft.com/azure-depth-platform/understanding-indirect-tof-depth-sensing/>`__,
`Melexis <https://media.melexis.com/-/media/files/documents/application-notes/time-of-flight-basics-application-note-melexis.pdf>`__,
`TI <https://www.ti.com/lit/wp/sloa190b/sloa190b.pdf?ts=1657842732275&ref_url=https%253A%252F%252Fwww.google.com%252F>`__.

.. _tof-control-id:

Time of Flight Camera Control IDs
=================================

``V4L2_CID_TOF_CLASS (class)``
    The TOF class descriptor. Calling :ref:`VIDIOC_QUERYCTRL` for
    this control will return a description of this control class.

``V4L2_CID_TOF_PHASE_SEQ (dynamic array u16)``
    Change the shift between illumination and sampling for each phase
    in degrees. The distance / amplitude (confidence) pictures are obtained
    by merging 3..8 captures of the same scene using different phase shifts
    (some TOF sensors use different frequency modulations).

    The size of dynamic array specify the number of captures.
    Also driver may decide whether V4L2_CID_TOF_FREQ_MOD and
    V4L2_CID_TOF_TIME_INTEGRATION should change the number
    of captures or rely on V4L2_CID_TOF_PHASE_SEQ control.
    The maximum size of the array is driver specific.

``V4L2_CID_TOF_FREQ_MOD (dynamic array u32)``
    The control sets the modulation frequency (in Hz) for each phase.
    The maximum array size is driver specific.

``V4L2_CID_TOF_TIME_INTEGRATION (dynamic array u16)``
    The control sets the integration time (in us) for each phase.
    The maximum array size is driver specific.
