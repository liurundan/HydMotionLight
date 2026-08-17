Manual | EN
TF5810
TwinCAT 3 | Hydraulic Positioning

2025-08-18 | Version: 1.8.3

Table of contents

Table of contents

1 Foreword .................................................................................................................................................... 7

1.1 Notes on the documentation .............................................................................................................  7

1.2

For your safety ..................................................................................................................................  7

1.3 Notes on information security............................................................................................................  9

2 Introduction to hydraulics ......................................................................................................................  10

3 General structure ....................................................................................................................................  15

3.1

3.2

3.3

Structure of the documentation .......................................................................................................  15

Functions, function blocks and types ..............................................................................................  16

The hydraulics library ......................................................................................................................  24

4 PLCopen Motion Control ........................................................................................................................  27

4.1

Administrative..................................................................................................................................  27

4.1.1

4.1.2

4.1.3

4.1.4

4.1.5

4.1.6

4.1.7

4.1.8

4.1.9

MC_Power_BkPlcMc .......................................................................................................  27

MC_ReadActualPosition_BkPlcMc ..................................................................................  29

MC_ReadActualTorque_BkPlcMc....................................................................................  30

MC_ReadActualVelocity_BkPlcMc ..................................................................................  31

MC_ReadAxisError_BkPlcMc ..........................................................................................  32

MC_ReadBoolParameter_BkPlcMc .................................................................................  33

MC_ReadDigitalOutput_BkPlcMc ....................................................................................  34

MC_ReadParameter_BkPlcMc ........................................................................................  36

MC_ReadStatus_BkPlcMc...............................................................................................  37

4.1.10 MC_Reset_BkPlcMc ........................................................................................................  39

4.1.11 MC_ResetAndStop_BkPlcMc ..........................................................................................  40

4.1.12 MC_SetOverride_BkPlcMc ..............................................................................................  41

4.1.13 MC_SetPosition_BkPlcMc ...............................................................................................  43

4.1.14 MC_SetReferenceFlag_BkPlcMc.....................................................................................  45

4.1.15 MC_WriteBoolParameter_BkPlcMc .................................................................................  46

4.1.16 MC_WriteDigitalOutput_BkPlcMc ....................................................................................  47

4.1.17 MC_WriteParameter_BkPlcMc ........................................................................................  48

4.2 Motion ............................................................................................................................................. 49

4.2.1

4.2.2

4.2.3

4.2.4

4.2.5

4.2.6

4.2.7

4.2.8

4.2.9

MC_CamIn_BkPlcMc .......................................................................................................  49

MC_CamOut_BkPlcMc ....................................................................................................  51

MC_CamTableSelect_BkPlcMc .......................................................................................  53

MC_DigitalCamSwitch_BkPlcMc .....................................................................................  54

MC_EmergencyStop_BkPlcMc ........................................................................................  57

MC_FlyingGear_BkPlcMc ................................................................................................  59

MC_GearIn_BkPlcMc.......................................................................................................  63

MC_GearInPos_BkPlcMc ................................................................................................  65

MC_GearOut_BkPlcMc....................................................................................................  67

4.2.10 MC_Home_BkPlcMc ........................................................................................................  68

4.2.11 MC_Halt_BkPlcMc ...........................................................................................................  71

4.2.12 MC_ImediateStop_BkPlcMc ............................................................................................  72

4.2.13 MC_MoveAbsolute_BkPlcMc...........................................................................................  73

4.2.14 MC_MoveJoySticked_BkPlcMc .......................................................................................  75

TF5810

Version: 1.8.3

3

Table of contents

4.2.15 MC_MoveRelative_BkPlcMc............................................................................................  77

4.2.16 MC_MoveVelocity_BkPlcMc ............................................................................................  79

4.2.17 MC_RampedStop_BkPlcMc.............................................................................................  80

4.2.18 MC_Stop_BkPlcMc ..........................................................................................................  82

4.2.19 MC_MoveJog_BkPlcMc ...................................................................................................  84

4.3 Data types .......................................................................................................................................  86

4.3.1

4.3.2

4.3.3

4.3.4

4.3.5

4.3.6

4.3.7

4.3.8

4.3.9

AXIS_REF_BkPlcMc........................................................................................................  86

E_TcPlcBufferedCmdType_BkPlcMc...............................................................................  89

E_TcMcCurrentStep.........................................................................................................  90

E_TcMcDriveType............................................................................................................  94

E_TcMcEncoderType.......................................................................................................  98

E_TcMCFbState.............................................................................................................  103

E_TcMcHomingType......................................................................................................  103

E_TcMCParameter ........................................................................................................  104

E_TcMcProfileType........................................................................................................  118

4.3.10

E_TcMcPressureReadingMode .....................................................................................  119

4.3.11

E_TcMcValveType .........................................................................................................  120

4.3.12 MC_BufferMode_BkPlcMc .............................................................................................  121

4.3.13 MC_CAM_ID_BkPlcMc ..................................................................................................  122

4.3.14 MC_CAM_REF_BkPlcMc ..............................................................................................  123

4.3.15 MC_CAMSWITCH_REF_BkPlcMc ................................................................................  123

4.3.16 MC_Direction_BkPlcMc .................................................................................................  124

4.3.17 MC_HomingMode_BkPlcMc ..........................................................................................  125

4.3.18 MC_StartMode_BkPlcMc ...............................................................................................  125

4.3.19 MC_TRACK_REF_BkPlcMc ..........................................................................................  126

4.3.20 OUTPUT_REF_BkPlcMc ...............................................................................................  126

4.3.21

ST_FunctionGeneratorFD_BkPlcMc..............................................................................  127

4.3.22

ST_FunctionGeneratorTB_BkPlcMc ..............................................................................  128

4.3.23

ST_TcMcAutoIdent ........................................................................................................  128

4.3.24

ST_TcHydAxParam .......................................................................................................  130

4.3.25

ST_TcHydAxRtData.......................................................................................................  141

4.3.26

ST_TcMcAuxDataLabels ...............................................................................................  149

4.3.27

ST_TcPlcDeviceInput.....................................................................................................  149

4.3.28

ST_TcPlcDeviceOutput..................................................................................................  153

4.3.29

ST_TcPlcMcLogBuffer ...................................................................................................  156

4.3.30

ST_TcPlcMcLogEntry ....................................................................................................  156

4.3.31

ST_TcPlcRegDataItem ..................................................................................................  157

4.3.32

ST_TcPlcRegDataTable ................................................................................................  157

4.3.33

ST_TcHybridAxParam ...................................................................................................  158

4.3.34

ST_TcHybridAxRtData...................................................................................................  161

4.3.35

ST_TcPlcInputAnalog ....................................................................................................  164

4.3.36

ST_TcPctrlParam...........................................................................................................  164

4.3.37 MC_Ref_Signal_Ref_BkPlcMc ......................................................................................  165

4.3.38

E_TcMcJogMode ...........................................................................................................  165

4.4

System .......................................................................................................................................... 166

4.4.1

Controller........................................................................................................................  166

4

Version: 1.8.3

TF5810

Table of contents

4.4.2

4.4.3

4.4.4

4.4.5

4.4.6

4.4.7

4.4.8

4.4.9

Drive...............................................................................................................................  187

Encoder..........................................................................................................................  198

FunctionGenerator .........................................................................................................  226

TableFunctions...............................................................................................................  229

Generators .....................................................................................................................  237

Runtime..........................................................................................................................  244

Message logging ............................................................................................................  256

Utilities............................................................................................................................  261

4.5

Parameter .....................................................................................................................................  279

4.5.1

4.5.2

4.5.3

4.5.4

4.5.5

4.5.6

4.5.7

4.5.8

4.5.9

MC_AxAdsCommServer_BkPlcMc ................................................................................  279

MC_Communications_BkPlcMc.....................................................................................  281

MC_AxAdsPtrArrCommServer_BkPlcMc.......................................................................  281

MC_AxAdsReadDecoder_BkPlcMc ...............................................................................  283

MC_AxAdsWriteDecoder_BkPlcMc ...............................................................................  285

MC_AxParamAuxLabelsLoad_BkPlcMc ........................................................................  286

MC_AxParamLoad_BkPlcMc.........................................................................................  287

MC_AxParamSave_BkPlcMc.........................................................................................  288

MC_AxUtiReadCoeDriveTerm_BkPlcMc .......................................................................  289

4.5.10 MC_AxUtiReadCoeEncTerm_BkPlcMc .........................................................................  291

4.5.11 MC_AxUtiReadRegDriveTerm_BkPlcMc .......................................................................  293

4.5.12 MC_AxUtiReadRegEncTerm_BkPlcMc .........................................................................  294

4.5.13 MC_AxUtiUpdateRegDriveTerm_BkPlcMc ....................................................................  295

4.5.14 MC_AxUtiUpdateRegEncTerm_BkPlcMc ......................................................................  297

4.5.15 MC_AxUtiWriteCoeDriveTerm_BkPlcMc .......................................................................  299

4.5.16 MC_AxUtiWriteCoeEncTerm_BkPlcMc .........................................................................  300

4.5.17 MC_AxUtiWriteRegDriveTerm_BkPlcMc .......................................................................  302

4.5.18 MC_AxUtiWriteRegEncTerm_BkPlcMc .........................................................................  303

4.6

Part 5 Homing ...............................................................................................................................  305

4.6.1

4.6.2

FinalizingFunctions ........................................................................................................  305

StepFunctions ................................................................................................................  308

5 Knowledge Base ...................................................................................................................................  320

5.1

FAQs ............................................................................................................................................. 321

5.2 Global constants ...........................................................................................................................  338

5.3

5.4

Valve ............................................................................................................................................. 347

Electric/hydraulic hybrid axes........................................................................................................  349

5.5 Configuration of an axis ................................................................................................................  363

5.5.1

FB_Power ......................................................................................................................  370

5.6

5.7

The PlcMcManager .......................................................................................................................  370

Sample programs ..........................................................................................................................  374

5.8 Commissioning..............................................................................................................................  380

5.8.1

5.8.2

5.8.3

5.8.4

5.8.5

5.8.6

Basic settings .................................................................................................................  380

Temporary zero compensation ......................................................................................  381

Movement directions ......................................................................................................  381

Position measuring system ............................................................................................  382

Characteristic curve measurement ................................................................................  382

Overlap...........................................................................................................................  383

TF5810

Version: 1.8.3

5

Table of contents

5.8.7

5.8.8

5.8.9

Reference velocity/velocity ratio.....................................................................................  383

Referencing....................................................................................................................  385

Dynamics/target approach .............................................................................................  385

6 Support and Service .............................................................................................................................  387

6

Version: 1.8.3

TF5810

Foreword

1 Foreword

1.1

Notes on the documentation

This description is intended exclusively for trained specialists in control and automation technology who are
familiar with the applicable national standards.
The documentation and the following notes and explanations must be complied with when installing and
commissioning the components.
The trained specialists must always use the current valid documentation.

The trained specialists must ensure that the application and use of the products described is in line with all
safety requirements, including all relevant laws, regulations, guidelines, and standards.

Disclaimer

The documentation has been compiled with care. The products described are, however, constantly under
development.
We reserve the right to revise and change the documentation at any time and without notice.
Claims to modify products that have already been supplied may not be made on the basis of the data,
diagrams, and descriptions in this documentation.

Trademarks

Beckhoff®, ATRO® , EtherCAT®, EtherCAT G®, EtherCAT G10®, EtherCAT P®, MX-System®, Safety over
EtherCAT®, TC/BSD®, TwinCAT®, TwinCAT/BSD®, TwinSAFE®, XFC®, XPlanar®, and XTS® are registered
and licensed trademarks of Beckhoff Automation GmbH.
If third parties make use of the designations or trademarks contained in this publication for their own
purposes, this could infringe upon the rights of the owners of the said designations.

EtherCAT® is a registered trademark and patented technology, licensed by Beckhoff Automation GmbH,
Germany.

Copyright

© Beckhoff Automation GmbH & Co. KG, Germany.
The distribution and reproduction of this document, as well as the use and communication of its contents
without express authorization, are prohibited.
Offenders will be held liable for the payment of damages. All rights reserved in the event that a patent, utility
model, or design are registered.

Third-party trademarks

Trademarks of third parties may be used in this documentation. You can find the trademark notices here:
https://www.beckhoff.com/trademarks.

1.2

For your safety

Safety regulations

Read the following explanations for your safety.
Always observe and follow product-specific safety instructions, which you may find at the appropriate places
in this document.

Exclusion of liability

All the components are supplied in particular hardware and software configurations which are appropriate for
the application. Modifications to hardware or software configurations other than those described in the
documentation are not permitted, and nullify the liability of Beckhoff Automation GmbH & Co. KG.

TF5810

Version: 1.8.3

7

Foreword

Personnel qualification

This description is only intended for trained specialists in control, automation, and drive technology who are
familiar with the applicable national standards.

Signal words

The signal words used in the documentation are classified below. In order to prevent injury and damage to
persons and property, read and follow the safety and warning notices.

Personal injury warnings

Hazard with high risk of death or serious injury.

 DANGER

 WARNING

Hazard with medium risk of death or serious injury.

There is a low-risk hazard that could result in medium or minor injury.

 CAUTION

Warning of damage to property or environment

The environment, equipment, or data may be damaged.

NOTICE

Information on handling the product

This information includes, for example:
recommendations for action, assistance or further information on the product.

8

Version: 1.8.3

TF5810

Foreword

1.3

Notes on information security

The products of Beckhoff Automation GmbH & Co. KG (Beckhoff), insofar as they can be accessed online,
are equipped with security functions that support the secure operation of plants, systems, machines and
networks. Despite the security functions, the creation, implementation and constant updating of a holistic
security concept for the operation are necessary to protect the respective plant, system, machine and
networks against cyber threats. The products sold by Beckhoff are only part of the overall security concept.
The customer is responsible for preventing unauthorized access by third parties to its equipment, systems,
machines and networks. The latter should be connected to the corporate network or the Internet only if
appropriate protective measures have been set up.

In addition, the recommendations from Beckhoff regarding appropriate protective measures should be
observed. Further information regarding information security and industrial security can be found in our
https://www.beckhoff.com/secguide.

Beckhoff products and solutions undergo continuous further development. This also applies to security
functions. In light of this continuous further development, Beckhoff expressly recommends that the products
are kept up to date at all times and that updates are installed for the products once they have been made
available. Using outdated or unsupported product versions can increase the risk of cyber threats.

To stay informed about information security for Beckhoff products, subscribe to the RSS feed at https://
www.beckhoff.com/secinfo.

TF5810

Version: 1.8.3

9

Introduction to hydraulics

2

Introduction to hydraulics

Hydraulics vs electromechanics: a technology comparison

Hydraulic drives differ from electric drives in that they have a fundamentally different design, so that their
behavior is only comparable to a limited degree. This special behavior and the distinctly different fields of
application require adapted control and monitoring mechanisms. The following table provides an overview of
these differences.

The electromechanical axes controlled by TwinCAT NC/NCI/CNC typically consist of an AX servo drive and
an AM synchronous motor with integrated position measuring system. The differences mainly relate to the
design, since linear or asynchronous motors can also be traced back to this basic principle. The servo drive
generates a rotating or moving magnetic field through the currents it controls, which is followed by the
moving part of the motor. The strength, speed and angular/rotational speed difference of this magnetic field
to the rotor is controlled in such a way that the desired movement is achieved. With appropriate design, a
configuration is created that can be easily modeled. Since the basic structure is constant, this basically also
applies to the model.

Hydraulic axes are a much more varied in terms of their design. In addition to the various variants of linear
cylinders (plungers, synchronous, differential, area-switchable cylinders etc.), several rotary drives (swivel
cylinders, rotary cylinders, various types of hydraulic motors) are available as actuators. The velocity can be
defined through continuous valves or primary or secondary controlled pumps. In addition, there are various
hydraulic circuits in which further components influencing the amount of oil or pressure are added. Most of
these have a non-linear or situation-dependent behavior.

Ultimately, these differences mean that applications which can be achieved by a precisely defined and then
precisely executed movement are nowadays largely realized electromechanically. The more complex, less
standardized and difficult to handle hydraulic axes are used for tasks in which their particular strengths can
be exploited. For example, they are ideally suited for applying large forces and energies over long periods or
in applications where space is limited. In many cases, the behavior they are used to controlled is atypical for
electromechanical drives, such as limiting or relieving pressure or force control. The plastics industry and
metal forming are just two examples.

Electric/hydraulic hybrid axes

Electromechanical servo axes and hydraulic axes both offer specific advantages. The combination of these
technologies creates a hybrid system that offers a new mixture of positive and negative properties from both
worlds. Even though it is not possible to utilize all advantages in this way while avoiding all disadvantages,
overall a clear advantage can be achieved by combining the technologies in a suitable manner. The
following section provides an overview of these concepts.

Proportional valve-controlled hydraulic axes are less efficient than servo axes, which is a significant
disadvantage. Their efficiency is limited by the principle of throttle control. Electric drive control based on the
PWM principle has been used for decades. For technical reasons (no switching valves with high flow rate
and low switching time << 1ms) this is not possible for hydraulic axes. In hybrid axes the oil flow is controlled
by changing the speed and possibly the direction of rotation of a constant current pump with a servo drive,
rather than by using a variable throttle. In theory, there is no pressure drop between the pump and the
cylinder. The pump can be regarded as a friction-locked but not form-locked gear unit, while the cylinder
assumes the role of a spindle.

A selectable feed constant can be made available by making provision for changing the effective cylinder
areas or the quantity of oil pumped per revolution by switching the oil paths depending on the situation. The
result is a true gear shift that is not available for an electromechanical axis. In applications that require
alternating high velocity and high power, this can lead to considerable savings.

Switching valves can be used to hydraulically fix a force once it has built up and relieve the load on the
electric drive. In this way, the torque reduction of an electromechanical axis can be avoided.

All components of the hybrid axis can be assembled as a self-contained module up to performance values
that can be quite considerable. In this case, all hydraulic connections are encapsulated internally, and the
only external connections are electrical ones. The axis is mountable and also exchangeable like an
electromechanical axis. In situations where higher performance is required, a conventional discrete structure
has to be used. However, it should be noted that a comparable electromechanical axis is also anything but
compact or light.

10

Version: 1.8.3

TF5810

Introduction to hydraulics

Further details on the configuration concept and commissioning can be found in the Knowledge Base (in
preparation).

Overview of differences

The differences in design described above have a considerable effect on the operating behavior of hydraulic
and electric drives. An overview of these effects is presented below.

Typical natural frequencies of electromechanical axes are in the range >80 Hz. Values below 20 Hz are not
uncommon for hydraulic axes. In both technologies, axes with >200 Hz can be realized, but for technical
and/or calculation reasons they are only used when necessary. The natural frequency has a direct influence
on controllability, since it limits the usable kP of the position controller. The controllability of
electromechanical axes is a prerequisite for standard NCs.

• For hydraulic axes, differential cylinders with just one piston rod are preferred. This makes the feed
constant (here defined as travel per oil quantity) direction-dependent. Standard NCs do not take this
behavior into account, because there is no such effect with electromechanical axes.

• The asymmetrical working surfaces of a differential cylinder require an asymmetrical pressure
distribution on the surfaces for a standstill in force equilibrium. If the axis starts in the opposite
direction, a different pressure distribution must be established. For this purpose, an amount of oil has
to be transported through the valves, which are initially only slightly opened, without any movement
taking place. This leads to a delayed startup. A comparable but much fiercer phenomenon occurs if the
axis has built up a pressing force beforehand. Standard NCs do not take this behavior into account,
because there is no such effect with electromechanical axes.

• Hydraulic actuators rely on seals to separate their workspaces from each other and from ambient.

These seals, which in some cases have long circumferential edges, are in contact with metal surfaces
and must slide on them. Above all, the transition from standstill to movement is accompanied by
pronounced changes in adhesion/sliding friction. The comparable effects with electromechanical axes
are several orders of magnitude smaller and are usually negligible. In the case of hydraulic axes, they
play a key role in determining the behavior on startup, when approaching the target and when moving
at low speeds.

• Hydraulic axes use continuously adjustable valves or pumps as actuators. These components are

always more or less non-linear. The system gain to be taken into account by the controller and the feed
constant to be used by the pilot control are dependent on the operating point. Compromises in motion
control can be reduced through linearization, but not completely avoided. Standard NCs do not take
this behavior into account, because there is no such effect with electromechanical axes.

• A dead range around the zero point of several 10 % of full scale is not uncommon for valves. Even with
linearization, position control at standstill is then only possible to a limited extent. Standard NCs do not
take this behavior into account, because there is no such effect with electromechanical axes.

• The output value sent to the valve defines the slider position and thus, via a non-linear mechanical
function, the openings for the oil flow. However, the pressure drop across the opening has a strong
influence on the actual oil quantity and thus on the cylinder speed. Fluctuations in the supply pressure
or cylinder pressure (resulting from the process force) have a strong influence on the axis velocity.

• It is not easily possible to use of an I component in the controller. In combination with the adhesion/

sliding friction changes, low-frequency oscillations can easily occur, which are difficult to control. The
cylinder oscillates periodically around positions determined by the working cycle, resulting in damage
to seals and surfaces in the medium term.

It may be possible to operate a hydraulic axis with a standard NC. The higher the quality of the component
selection and configuration, the easier it is to do this. However, expectations regarding the behavior then
offer little room for compromise. Conventional hydraulic axis configurations usually require adapted
solutions, which are provided by Beckhoff Automation in the hydraulic library.

Motion Control in a different way

The key function of a Motion Control solution is the set value generator. It calculates or resolves
instantaneous set values for position, velocity, acceleration and possibly jerk. The time-controlled mode of
operation of the NC is well known in this context. However, there is an often overlooked alternative that is of
particular interest for hydraulic axes. Its derivation and the differences are described below.

TF5810

Version: 1.8.3

11

Introduction to hydraulics

A set value generator can operate either as a function of or independently of the variables of another axis.
The former is the case if the values for a cam plate coupling are derived from the values of another axis via a
table or, in the case of a gear coupling, via a calculation formula. This requires a position controller that is
active during the motion. Both the hydraulics library and, above all, the NC offer various options here.

If the values are calculated independently of other axes, a distinction must be made between time-based and
displacement-based generation. Like practically all current MC systems, TwinCAT NC/NCI/CNC works on a
time-controlled basis. The core technology of the hydraulic library is path-controlled, although here, too, time-
controlled operation is possible. The differences are shown below.

A time-controlled Motion Control solution uses equations in which the motion profile runs on a time basis.
This is shown below for an accelerated movement:

V := A * t

P := ½ * A * t2

If the first equation is squared and then both equations are resolved to t2 and equated, the following equation
is obtained:

V := SQRT( 2 * A * P )

If the absolute value of the remaining distance s to a target position is used for P and the sign is restored, a
suitable braking ramp results.

V := ± SQRT( 2 * A * ABS( s ) )

It should be noted that the time as the controlling variable has been replaced by the path. Combining this
braking ramp with a ramp for the acceleration phase and a constant phase provides the basis for a simple
but particularly robust Motion Control solution that is characterized by the following features:

• Delayed axis responses at the start of a motion are ignored. The valve is not initially opened

excessively and without effect by a position controller, only to be controlled back down again to a
standstill once the cylinder springs into action.

• No position control takes place even during the active motion. If the axis does not move at the correct

velocity or at varying velocity, this is automatically compensated for by a premature or delayed initiation
of the brake phase.

• Counter forces generated by the process slow down the axis. However, this inevitably leads to an

increase in pressure even without a reaction from the control unit, possibly up to the supply pressure
and thus to the maximum available force. If this is not sufficient for a further movement, it would not be
affected by a controller either. Even without position control, there is no risk of the axis stopping.

• When approaching the target position, the velocity is adjusted according to the remaining distance.

This adjustment happens continuously and thus compensates for inaccurate braking.

• Non-linearities are also compensated. However, they can appear as interfering irregularities in the

acceleration. In this case, the behavior can be improved by a more precise linearization.

• The permanently active position controller, which is inevitable with the time-controlled principle,
increases the tendency to oscillate and generates undesirable changes in the speeds. With
electromechanical axes, this effect is less pronounced and can be tolerated. Hydraulic axes are
subjected to considerably more excitation sources, and they have lower frequency and are less
attenuated. The effect is distinct and often rather troublesome.

• The accuracy at the target does not depend on the method used. In the time-controlled "vertical"

principle, a deviation of the axis behavior from the ideal is compensated by an added controller output.
With the displacement-based principle, the reaction takes place by "horizontal" stretching or
compressing of the profile.

• With the time-based principle, two axes that are operated with the same parameters and started

simultaneously with the same commands will move as if they were mechanically connected. Both axes
move at the right time in the right place and at the right velocity. The deviation is limited to the (typically
small) lag errors and is not integrated.

• With the displacement-based principle, influences from the process or even manufacturing tolerances
of the components are not compensated. Deviations are integrated within a movement. There is no
definitive expectation of a link between two axes that are operated with the same parameters and
started simultaneously with the same commands. They are positioned in the target with the achievable
accuracy, but do not necessarily arrive there at exactly the same time.

12

Version: 1.8.3

TF5810

Introduction to hydraulics

Structure of the library

In contrast to the NC, the library functions work entirely in the PLC runtime. This has several consequences,
which are listed below.

• Internal function blocks are usually also visible. This makes the online view less transparent. On the

other hand, local variables can be used for an analysis.

• All parameters and even runtime variables are visible and accessible. This creates opportunities for

specific manipulations. It should be obvious why this should be done with the utmost care.

• Nothing is done without a corresponding function block being called directly or indirectly. In contrast to
the NC, the internal operation of the Motion Control is very transparent. This is particularly true for:

◦ Loading and saving of parameters.

◦ Recording of actual values.

◦ Setpoint generation.

◦ Control.

◦ Output adjustment.

• In contrast to NC, there are no "finished" axes. This increases the initial work, but also offers

opportunities for producing adapted properties.

• Since the axis is configured in the PLC application, it is easily possible that unexpected and difficult to

understand effects are created by an incorrect sequence or combination of the called function blocks. It
is highly advisable to follow the examples.

• Since the function blocks are called by the PLC, the Motion Control also works with the cycle time of
the PLC task. A task with a typical NC cycle time of considerably less than 10 ms should be used.

In order to make the projects more transparent, the most important function blocks are implemented
according to the PLCopen standards. Among other things, this standard specifies that the function blocks are
linked to an axis via a reference named AxisRef. Since there is no hidden task level in the library, all data
(parameters, runtime values) required for the axis are integrated in this structure. The communication of the
function blocks of an axis is based on shared use of this reference. The only exceptions are the signals
defined by PLCopen. The Execute input can be controlled by the Done output of another function block, for
example, in order to create a desired sequence.

Structure of an application

In a PLC application realized with the hydraulics library, a distinction should be made between three different
types of function block:

• System function blocks related to all axes. This includes communication with the PlcMcManager IBN
tool or handling of message recording. Regardless of the number of axes, these function blocks must
be instantiated exactly once per project and called up exactly once per cycle. Obviously, this should be
done from the Main() of a program.

• Function blocks used for the configuration of an axis. These include, for example, the encoder function
block and the setpoint generator etc. Exactly one instance of these function blocks must be created for
each axis. The call should be made exactly once per cycle.

• Function blocks related to an axis. These include, for example, the MC_MoveAbsolute_BkPlcMc

function block, the MC_Stop_BkPlcMc function block, etc. More than one instance of these function
blocks can be created per axis. As a rule, the call must be made exactly once per cycle.

If the application has only one axis, this difference is less clear, but must still be considered.

System function blocks

The system function blocks include the following:

• MC_AxAdsCommServer_BkPlcMc()

This function block provides an ADS connection for the PlcMcManager for all axes. If this function block is
not called cyclically, no connection is established.

• MC_AxRtLoggerSpool_BkPlcMc() or MC_AxRtLoggerDespool_BkPlcMc

TF5810

Version: 1.8.3

13

Introduction to hydraulics

This function block manages the message buffer. If exactly one of these function blocks is not called
cyclically, the message buffer overflows, and subsequent messages are lost.

As you can see, the system function blocks require access to all affected structures. At the same time, the
axis-related function blocks also require access. This can be easily ensured by creating the structures as
VAR_GLOBAL. This is shown in the examples and applies especially to:

• The axis references. They should be created as ARRAY[1... number of axes] OF Axis_Ref_BkPlcMc.

◦ This means that it is not possible to distribute the axis references in modules of the application.

◦ There is an alternative method that works with POINTER lists. Special care is required in his case.

This method is therefore not recommended for general use.

• The message buffer of type ST_TcPlcMcLogBuffer. The buffer is shared by all axes, and the

management function block therefore cannot be assigned to an axis.

Function blocks for the configuration of an axis

These always include:

• The initialization function block MC_AxUtiStandardInit_BkPlcMc().

• The function blocks of the actual value acquisition. These always include a function block of type

MC_AxRtEncoder_BkPlcMc() and one or more function blocks for determining pressures or forces, as
required. Filtering can be used, if necessary.

• A function block of type MC_AxRuntime_BkPlcMc() for setpoint generation. This function block

contains a standard position controller.

• A function block of type MC_AxAxRtFinish_BkPlcMc() or MC_AxRtFinishLinear_BkPlcMc. Various
output parameters are combined here, and a section-by-section or characteristic curve-controlled
output linearization is carried out.

• A function block of type MC_AxRtDrive_BkPlcMc() that adapts to the I/O variables of the output

hardware.

If necessary, this minimum structure must be supplemented by function blocks that give the axis additional
capabilities. These include, for example, function blocks for controlling pressures or forces, as an alternative
position controller or for automatic measurement of characteristic curves. To be effective, the calls of these
function blocks must be inserted at the correct position between the above-mentioned function blocks.

The transparency of the application can be improved by combining these function blocks into an axis block
with general interfaces.

Axis-related function blocks

These include the usual function blocks for configuring the working cycle of an axis.

• MC_Power_BkPlcMc

• MC_MoveAbsolute_BkPlcMc

• MC_Stop_BkPlcMc

• MC_Reset_BkPlcMc

• MC_Home_BkPlcMc

• MC_GearIn_BkPlcMc

• MC_GearOut_BkPlcMc

• etc.

Since the behavior of these function blocks corresponds to the PLCopen definitions, they can largely be
used like the corresponding function blocks of the TC_MC libraries. However, the function blocks of these
libraries only send commands to the NC driver and observe its reactions and feedback. Various function
blocks of the hydraulic library contain essential parts of the functionality and must be called continuously and
in every cycle. This must be taken into account when creating the application.

14

Version: 1.8.3

TF5810

General structure

3 General structure

3.1

Structure of the documentation

Each axis consists of an axis structure under the name "AXIS_REF_BkPlcMc", which is composed of
different external structures. This axis structure contains all the data (runtime data and parameter data) for
this axis.

Certain function blocks have to be present in each application, to enable an axis to move. These function
blocks include:

• MC_AxUtiStandardInit_BkPlcMc [} 254]: Initialization and monitoring of the different axis components.
Such an FB should be called cyclically. Blocks such as MC_Power_BkPlcMc, etc. may only be called
after successful initialization.

• MC_Power_BkPlcMc [} 27]: The function block is used to control an external actuator. The function
block issues release notifications to valve output stages or frequency converters, for example.

• MC_AxStandardBody_BkPlcMc [} 253]: In each case the function block calls a function block of type
MC_AxRtEncoder_BkPlcMc [} 198]: Determination of the actual position of the axis from the input
information of a hardware module.
MC_AxRuntime_BkPlcMc [} 237]: Deals with profile generation.
MC_AxRtFinish_BkPlcMc [} 246]: Adaptation of the control value to the special characteristics of the
axis (characteristic curve linearization)
MC_AxRtDrive_BkPlcMc [} 187]: The function block performs preparation of the control value for the
axis for it to be output on a hardware module.

• MC_AxAdsCommServer_BkPlcMc [} 279]: Establishes the connection to PlcMcManager and monitors it.
This block must be called independent of the initialization, in order to enable commissioning without
existing parameters.

Optional useful function blocks are:

• MC_AxRtLoggerSpool_BkPlcMc [} 261]: The function block prevents overflowing of the LogBuffer of the

library.

• MC_AxParamDelayedSave_BkPlcMc: Performs an auto-save of the axis parameters.

TF5810

Version: 1.8.3

15

General structure

The so-called "PlcMcManager" is provided for commissioning. This tool consolidates setting parameters and
is intended to facilitated commissioning of the system.

The first example is intended to illustrate the "first steps".

Function groups

Management functions [} 18]

Single axis motion functions [} 19]

Axis group motion functions [} 19]

Drive adjustments [} 19]

Encoder adjustments [} 19]

Parameter handling [} 20]

Motion generators [} 19]

Controller [} 21]

Table functions [} 21]

Message logging [} 21]

Runtime functions [} 22]
Data types

Description
Functions for management and monitoring of axes,
parameter access and states.
Triggering and monitoring of active movements for
individual axes.
Triggering and monitoring of active movements for
axis groups.
Function blocks for preparing axis control values for
output on output devices (terminals, actuators etc.)
in the periphery.
Function blocks for evaluating actual position data,
which were read by input devices (terminals,
encoders etc.) in the periphery.
Function blocks for saving, reading and
communicating parameters.
Control value generators for active axis movements

Controllers for various state variables: position,
velocity, pressure.
Table functions for non-linear mappings and cam
plates
Message recording.

Various runtime functions.

Enumerations [} 23] and structures [} 24] used in
the library

3.2

Functions, function blocks and types

Available from version 3.0

All the functions, function blocks and data types present in the library are listed here.

You will find answers to frequently asked questions and notes on the use of the library, setting up, problem
analysis and example projects in the Knowledge Base [} 320].

Some of the components listed here are not intended to be used by an application. Their presence, interface
and behavior is therefore not guaranteed. Because, however, a TwinCAT PLC library is strictly open, it is not
possible to hide these internal components. It is, nevertheless, essential to avoid calling these components,
identified with (internal use only) or (not recommended), directly from an application. If one of these
components would, in practice, be useful for you, please make contact with our Support Department. We will
then examine the possibility of making the function block available to you, independently of the library, and
for you to then take the responsibility for using it.

If the library contains function blocks, types or constants that are not listed in the documentation, then these
are elements that have not yet been approved, and are the subject of current software maintenance and
development work. These elements must never be directly used in an application, because they are, as a
general rule, not yet tested.

16

Version: 1.8.3

TF5810

General structure

The hydraulic library only offers a restricted range of functions, even in connection with electrical
drives. TwinCAT NC PTP, NC I and CNC offer a significantly broader spectrum and more
comprehensive support for commissioning and diagnosis.

A number of libraries are available, which deal with a typical axis configuration or special
functionalities. These libraries require the TcPlcHydraulics library and have to be ordered
separately.

Name
TcPlcLibHydraulics_30_2R2Vgantry.LIB
TcPlcLibHydraulics_30_4R3Vgantry.LIB

Description
in preparation
in preparation

PLC open Motion Control

The function blocks listed here are oriented towards:

Technical Specification

PLCopen - Technical Committee 2 - Task Force

Function blocks for motion control

Part 1 Version 1.1 and Part 2 Version 0.99F (definition not yet finalized)

The names of these function blocks begin with MC_ and end with _BkPlcMc.

Parts of the PLCopen definitions have not yet been finalized. Future versions of the library may be
subject to modifications.
Such modifications may relate to

• Names, behavior or even existence of functions, function blocks or derived data types

• Names, behavior, types or existence of input or output signals

TF5810

Version: 1.8.3

17

General structure

Administrative Function blocks

Name

MC_CamTableSelect_BkPlcMc [} 53]

MC_Power_BkPlcMc [} 27]

MC_ReadActualPosition_BkPlcMc [} 29]

MC_ReadActualTorque_BkPlcMc [} 30]

MC_ReadActualVelocity_BkPlcMc [} 31]

MC_ReadAxisError_BkPlcMc [} 32]

Description
The function block initializes a variable of type
ST_TcPlcMcCamId, thereby preparing a cam plate for
the coupling of two axes.
Function block to control an external actuator.

The actual position of an axis is determined.

The actual force or the actual pressure of an axis is
determined.
The actual velocity of an axis is determined.

The current error code of an axis is found.

MC_ReadBoolParameter_BkPlcMc [} 33]

The boolean parameters of an axis are read.

MC_ReadDigitalOutput_BkPlcMc [} 34]

MC_ReadParameter_BkPlcMc [} 36]

MC_ReadStatus_BkPlcMc [} 37]

MC_Reset_BkPlcMc [} 39]

MC_ResetAndStop_BkPlcMc [} 40]

MC_SetOverride_BkPlcMc [} 41]

MC_SetPosition_BkPlcMc [} 43]

MC_SetReferenceFlag_BkPlcMc [} 45]

MC_WriteBoolParameter_BkPlcMc [} 46]

MC_WriteDigitalOutput_BkPlcMc [} 47]

MC_WriteParameter_BkPlcMc [} 48]

The current state of a digital output of a cam controller is
determined.
The non-boolean parameters of an axis are read.

The state of the axis is decoded.

The axis is placed in a state ready for operation.

The axis is placed in a state ready for operation and is
stationary.
The axis override is set.

The actual position of the axis is set.

The referencing flag of the axis is defined. (Function is
not defined by PLCopen)
The boolean parameters of an axis are written.

The current state of a digital output of a cam controller is
defined.
The non-boolean parameters of an axis are written.

18

Version: 1.8.3

TF5810

Motion Function Blocks, Single Axis

Name

MC_DigitalCamSwitch_BkPlcMc [} 54]

MC_EmergencyStop_BkPlcMc [} 57]

MC_Halt_BkPlcMc [} 71]

MC_Home_BkPlcMc [} 68]

MC_ImediateStop_BkPlcMc [} 72]

MC_MoveAbsolute_BkPlcMc [} 73]

MC_MoveJoySticked_BkPlcMc [} 75]

MC_MoveRelative_BkPlcMc [} 77]

MC_MoveVelocity_BkPlcMc [} 79]

MC_RampedStop_BkPlcMc [} 80]

MC_Stop_BkPlcMc [} 82]

Motion Function blocks, Multiple Axis

Name

MC_CamIn_BkPlcMc [} 49]

MC_CamOut_BkPlcMc [} 51]

MC_GearIn_BkPlcMc [} 63]

MC_GearInPos_BkPlcMc [} 65]

MC_GearOut_BkPlcMc [} 67]

System Function Blocks

Name

MC_AxRtDrive_BkPlcMc [} 187]

MC_AxRtEncoder_BkPlcMc [} 198]

MC_AxRtFinish_BkPlcMc [} 246]

MC_AxRtFinishLinear_BkPlcMc [} 247]

MC_AxRuntime_BkPlcMc [} 243]

MC_AxRtGenerator_BkPlcMc [} 237]

MC_AxRtController_BkPlcMc [} 245]

General structure

Description
Generation of software cams as a function of position,
direction of movement and velocity of an axis.
Stopping a movement without reaching the target
position. (Function is not defined by PLCopen)
Stopping a movement without reaching the target
position.
Initiation and monitoring of homing.

Stopping a movement without reaching the target
position. (Function is not defined by PLCopen)
Start and monitoring of a positioning process at a
specifiable velocity to absolutely stated target co-
ordinates.
Starting and controlling of an axis movement with a
proportional control unit. (Function is not defined by
PLCopen)
Start and monitoring of a positioning process at a
specifiable velocity over an absolutely stated distance.
Start and monitoring of a positioning process at a
specifiable velocity but with no specified target.
Stopping of a movement with a pure time ramp.

Stopping a movement without reaching the target
position.

Description
The function block starts and monitors a cam plates
coupling between two axes.
The function block releases a cam plate coupling
between two axes.
Start and monitoring of the gear coupling of two axes.

On-the-fly gear coupling of two axes.

Cancelling the gear coupling of two axes.

Description
Preparation of the control value of the axis for output on a
hardware module, mapping information.
Determination of the actual position of the axis from the
input information of a hardware module, mapping
information.
Adaptation of the generated control value to the special
features of the axis.
Adjustment of the generated control value to the special
features of the axis, taking into account a characteristic
curve.
Position value generation and position control of the axis.

Control value generation for the axis.

Position control of the axis.

TF5810

Version: 1.8.3

19

General structure

System function blocks, other actual values

Name

MC_AxRtReadForceDiff_BkPlcMc [} 215]

Description
Determination of the differential actual force of an axis.

MC_AxRtReadForceSingle_BkPlcMc [} 218]

Determination of the one-sided actual force of an axis.

MC_AxRtReadPressureDiff_BkPlcMc [} 220]

MC_AxRtReadPressureSingle_BkPlcMc [} 222]

Determination of the differential actual pressure of an
axis.
Determination of the one-sided actual pressure of an
axis.

System Function Blocks, Parameter

Name

MC_AxAdsCommServer_BkPlcMc [} 279]

MC_AxAdsReadDecoder_BkPlcMc [} 283]

MC_AxAdsWriteDecoder_BkPlcMc [} 285]

MC_AxAdsPtrArrCommServer_BkPlcMc [} 281]

MC_AxParamAuxLabelsLoad_BkPlcMc [} 286]

MC_AxParamLoad_BkPlcMc [} 287]

MC_AxParamSave_BkPlcMc [} 288]

Description
The application is given the capacity to function as an
ADS server.
The function block decodes ADS read accesses for an
ADS server.
The function block decodes ADS write accesses for an
ADS server.
The application is given the capacity to function as an
ADS server.
Loading the label texts for the client-specific axis
parameters from a file.
Load the parameters for an axis from a file.

Write the parameters for an axis into a file.

MC_AxParamDelayedSave_BkPlcMc [} 261]

Delayed writing of the axis parameters.

MC_AxUtiReadCoeDriveTerm_BkPlcMc [} 289]

MC_AxUtiReadCoeEncTerm_BkPlcMc [} 291]

MC_AxUtiReadRegDriveTerm_BkPlcMc [} 293]

MC_AxUtiReadRegEncTerm_BkPlcMc [} 294]

Reading the contents of a register from the EL terminal,
which is used as drive interface for the axis.
Reading the contents of a register from the EL terminal,
which is used as encoder interface for the axis.
Reading the contents of a register from the KL terminal,
which is used as drive interface for the axis.
Reading the contents of a register from the KL terminal,
which is used as encoder interface for the axis.

MC_AxUtiUpdateRegDriveTerm_BkPlcMc [} 295] Writing a parameter set into the register of a KL terminal,

MC_AxUtiUpdateRegEncTerm_BkPlcMc [} 297]

MC_AxUtiWriteCoeDriveTerm_BkPlcMc [} 299]

MC_AxUtiWriteCoeEncTerm_BkPlcMc [} 300]

MC_AxUtiWriteRegDriveTerm_BkPlcMc [} 302]

MC_AxUtiWriteRegEncTerm_BkPlcMc [} 303]

MC_LinTableExportToAsciFile_BkPlcMc [} 275]

MC_LinTableExportToBinFile_BkPlcMc [} 276]

MC_LinTableImportFromBinFile_BkPlcMc [} 278]

which is used as drive interface for the axis.
Writing a parameter set into the register of a KL terminal,
which is used as encoder interface for the axis.
Writing the contents of a register into the EL terminal,
which is used as drive interface for the axis.
Writing the contents of a register into the EL terminal,
which is used as encoder interface for the axis.
Writing the contents of a register into the KL terminal,
which is used as drive interface for the axis.
Writing the contents of a register into the KL terminal,
which is used as encoder interface for the axis.
The function block exports a linearization table to a file in
ASCI format.
The function block exports a linearization table to a file in
binary format.

in ASCI format.
The function block imports a linearization table from a file
in binary format.

MC_LinTableImportFromAsciFile_BkPlcMc [} 277] The function block imports a linearization table from a file

20

Version: 1.8.3

TF5810

System Function Blocks, Controllers

Name

MC_AxCtrlAutoZero_BkPlcMc [} 166]

MC_AxCtrlPressure_BkPlcMc [} 172]

MC_AxCtrlPressureFF_Ex_BkPlcMc [} 176]

General structure

Description
Automatic zero balance.

Controller for pressure build-up control.

Extended controller for a pressure controller with a build-
up action.
Controller for pressure displacement control.

MC_AxCtrlPullbackOnPressure_BkPlcMc
MC_AxCtrlSlowDownOnPressure_BkPlcMc [} 178] Controller for pressure relief control.

MC_AxCtrlStepperDeStall_BkPlcMc [} 182]
MC_AxCtrlVelocity_BkPlcMc
MC_AxCtrlVeloMoving_BkPlcMc

Monitoring the movement of a stepper motor axis.

Controller for the axis velocity.
Controller for the axis velocity.

System Function blocks, TableFunctions

Name

MC_AxTableFromAsciFile_BkPlcMc [} 229]

Description
Reading the content of table from a text file.

MC_AxTableFromBinFile_BkPlcMc [} 230]

Reading the content of table from a binary file.

MC_AxTableReadOutNonCyclic_BkPlcMc [} 232]

MC_AxTableToAsciFile_BkPlcMc [} 234]

MC_AxTableToBinFile_BkPlcMc [} 235]

Function block for determining the slave values assigned
to a master value with the aid of a table.
Writing the contents of a table to text file.

Writing the contents of a table to a binary file.

System Function blocks, Message Logging

Name

MC_AxRtLogAxisEntry_BkPlcMc [} 256]

MC_AxRtLogClear_BkPlcMc [} 257]

MC_AxRtLogEntry_BkPlcMc [} 258]

MC_AxRtLoggerDespool_BkPlcMc [} 259]

MC_AxRtLoggerRead_BkPlcMc [} 260]

MC_AxRtLoggerSpool_BkPlcMc [} 261]

Description
An axis-related message is entered in the LogBuffer of
the library.
Clear and initialize all entries in the LogBuffer.

A message is entered in the LogBuffer of the library.

Ensure the minimum number of free messages in the
LogBuffer of the library.
Reading a message from the LogBuffer of the library.

Transferring messages from the LogBuffer of the library
into the Windows event viewer.

TF5810

Version: 1.8.3

21

General structure

System function blocks, runtime functions

Name

MC_AxRtCheckSyncDistance_BkPlcMc [} 244]

MC_AxRtCmdBufferExecute_BkPlcMc [} 256]

MC_AxRtCommandsLocked_BkPlcMc [} 262]

MC_AxRtGoErrorState_BkPlcMc [} 249]

Description
Monitoring of the distance between the referencing cam
and zero pulse.
Processing of the command buffer.

The function simplifies setting and deleting of a protective
function in the status double word of an axis.
(not recommended) The axis is placed into an error state.

MC_AxRtMoveChecking_BkPlcMc [} 250]

Monitoring the movement of an axis.

MC_AxRtSetDirectOutput_BkPlcMc [} 251]

Direct output of a control value.

MC_AxRtSetExtGenValues_BkPlcMc [} 252]

MC_AxStandardBody_BkPlcMc [} 253]

MC_AxUtiAutoIdent_BkPlcMc
MC_AxUtiAutoIdentSlave_BkPlcMc

MC_AxUtiAverageDerivative_BkPlcMc [} 265]

MC_AxUtiPT1_BkPlcMc [} 267]

MC_AxUtiPT2_BkPlcMc [} 267]

Supplying an axis with command variables, which do not
originate from the axis' own generator.
Calls the usual sub-components for an axis (encoder,
generator, finish, drive).
Automatic determination of axis parameters.
in preparation: Automatic determination of slave axis
parameters.
Determination of the derivative of value through numeric
differentiation over than one cycle.
Calculation of a first-order low-pass.

Calculation of a second-order low-pass.

MC_AxUtiSlewRateLimitter_BkPlcMc [} 268]

Generation of a rise-limited ramp.

MC_AxUtiSlidingAverage_BkPlcMc [} 269]

Determination of a sliding average value.

MC_AxUtiStandardInit_BkPlcMc [} 254]

Initialization and monitoring of axis components.

MC_FunctionGeneratorFD_BkPlcMc [} 226]

A function generator.

MC_FunctionGeneratorSetFrq_BkPlcMc [} 227]

MC_FunctionGeneratorTB_BkPlcMc [} 228]

Updates the operating frequency of a time base for one
or several function generators.
Updates a time base for one or several function
generators.

22

Version: 1.8.3

TF5810

Data types: Enumerations

Name

E_TcMcCurrentStep [} 90]

E_TcMcDriveType [} 94]

E_TcMcEncoderType [} 98]

E_TcMCFbState [} 103]

E_TcMcHomingType [} 103]

E_TcMCParameter [} 104]

E_TcMcPressureReadingMode [} 119]

E_TcMcProfileType [} 118]

E_TcPlcBufferedCmdType_BkPlcMc [} 89]

MC_BufferMode_BkPlcMc [} 121]

MC_Direction_BkPlcMc [} 124]

MC_HomingMode_BkPlcMc [} 125]

MC_StartMode_BkPlcMc [} 125]

General structure

Description
This enumeration returns codes for the internal states of
the control value generators.
The constants in this enumeration are used to identify the
hardware used to output the control values for an axis.
The constants in this enumeration are used to identify the
hardware used to acquire the actual values for an axis.
This enumeration supplies codes for the current state of
an axis.
This enumeration supplies codes for the referencing
method used by an axis.
The constants listed here are used for numbering
parameters.
The constants in this list determine which actual value in
the ST_TcHydAxRtData structure of the axis is to be
updated with the result of a pressure or force
measurement.
The constants listed here are used for identifying control
value generators.
In preparation: The constants in this list are used to
identify buffered axis commands.
The constants in this list are used for controlling blending
according to PLC Open.
This enumeration supplies codes for the direction of
movement if this information is not contained in other
data or cannot be in determined on the basis of the
situation.
This enumeration returns codes for specification of the
referencing method.
The constants in this list are used for identifying the
modes during axis startups.

TF5810

Version: 1.8.3

23

General structure

Data types: Structures

Name

Axis_Ref_BkPlcMc [} 86]

CAMSWITCH_REF_BkPlcMc [} 123]

MC_CAM_ID_BkPlcMc [} 122]

MC_CAM_REF_BkPlcMc [} 123]

OUTPUT_REF_BkPlcMc [} 126]

ST_FunctionGeneratorFD_BkPlcMc [} 127]

ST_FunctionGeneratorTB_BkPlcMc [} 128]

ST_TcMcAutoIdent [} 128]

ST_TcMcAuxDataLabels [} 149]

ST_TcHydAxParam [} 130]

ST_TcHydAxRtData [} 141]

ST_TcPlcMcLogBuffer [} 156]

ST_TcPlcMcLogEntry [} 156]

ST_TcPlcDeviceInput [} 149]

ST_TcPlcDeviceOutput [} 153]

ST_TcPlcRegDataItem [} 157]

ST_TcPlcRegDataTable [} 157]

TRACK_REF_BkPlcMc [} 126]

Description
A variable of this type contains all the necessary
variables or pointers to variables that are associated with
an axis.
A variable of this type is transferred to an
MC_DigitalCamSwitch_BkPlcMc [} 54] function block.
A variable of this type contains the description of a cam
plate prepared for coupling.
A variable of this type contains the description of a
provided cam plate.
A variable of this type contains output data of an
MC_DigitalCamSwitch_BkPlcMc [} 54] function block.
A variable of this type contains parameters for defining
the output signals of a function generator.
A variable of this type contains parameter for defining a
time base for a function generator.
A variable of this type contains the parameters for an
MC_AxUtiAutoIdent_BkPlcMc function block.
A variable of this type contains label texts for the client-
specific axis parameters.
A variable of this type contains all the parameters for an
axis.
A variable of this type contains the runtime data for an
axis.
A variable with this structure forms the LogBuffer of the
library.
A variable with this structure contains a message of the
LogBuffer of the library.
This structure contains the input image variables of an
axis.
This structure contains the output image variables of an
axis.
This structure contains a parameter set for a KL terminal.

This structure contains a parameter for a KL terminal.

In preparation.

3.3

The hydraulics library

Special control algorithms are required to meet the requirements of the hydraulic systems. The PLC libraries
TcPlcHydraulics_30 (for TwinCAT 2) and TC2_Hydraulic (for TwinCAT 3) contain a number of blocks and
functions for hydraulic axes and the data types used in them. They extend support for this drive technology
by enabling the operation of axes whose properties (limit frequency, scattering behavior) make them
unsuitable for position control, or whose tasks differ from those of electrical servo axes.

The product presented here includes:

• the software library "TcPlcHydraulics.lib" or "Tc2_Hydraulics.compiled-library"

• the commissioning tool "PlcMcManager.exe"

To simplify the use of the library, the function blocks are designed based on specifications by the IEC61131
user organization (PLCopen) and certified accordingly.

24

Version: 1.8.3

TF5810

General structure

The documentation for version V2.1 will continue to be available.

Library topics:

• Evaluation of encoders [} 198]

• Evaluation of pressure cells

• Various filter functions

◦ Pt1 filter

◦ Moving average [} 269]

◦ Rise limitation [} 268]

• Full access to internal parameters

• Motion control

• Controllers for

◦ Pressure/force

◦ Position

◦ Velocity

◦ Possibility of in-house controller development

• Synchronization of hydraulic and electric axes

• Adaptation of control values to output devices

• Full handling of complex devices

• Message logging

• Parameter handling

◦ Storage and loading routines

◦ Autosave

• Characteristic curve linearization

◦ Section by section

◦ Characteristic compensation curve

The following motion controllers are supported:

1. Time-based motion control:

◦ The controlling parameter for the profile generation is time.

◦ The generator does not “know” the axis.

◦ Only the pre-controlled position controller establishes the connection.

TF5810

Version: 1.8.3

25

General structure

2. Displacement-based motion control:

◦ The controlling parameter for the profile generation is the residual path.

◦ The generator “knows” the axis.

◦ During motion no position control is possible/required.

3. Dependent motion control:

◦ The set values are calculated from the values of another axis, based on a mapping rule (gear

formula, curve table).

◦ The generator does not “know” the axis.

◦ Only the pre-controlled position controller establishes the connection.

Displacement- and time-based motion control:

Time-based motion control uses time reference variable. The basic equations are

v=a*t and

s=0.5*a*t*t.

The set value generator provides a velocity and a position, which are evaluated by the position and velocity
controller and offset against the current position.

During displacement-controlled positioning, in contrast to time-controlled the control value for the axis is
calculated as a function of the residual path. Rearranging the above equations results in

v=sqrt(2*a*s).

Both methods have advantages and disadvantages.

• Time-controlled require closed-loop control, particularly for acceleration and deceleration processes.

The feedback is essential to enable the velocity controller to generate the correct output value.
However, such a control loop reacts strongly to stick/slip effects or supply pressure fluctuations, which
can cause the system to start oscillating.

• Displacement-controlled axes do not have to be operated in closed-loop control. This method is

therefore significantly more robust against external interference.

• Since displacement-control of axes is based on the displacement, not on the time, a velocity is

provided, but not readjusted. This makes the positioning of hydraulic axes very robust.

Both methods are supported by the hydraulics library and can also be used in combination.

26

Version: 1.8.3

TF5810

PLCopen Motion Control

4 PLCopen Motion Control

4.1

Administrative

4.1.1

MC_Power_BkPlcMc

Available from version 3.0

The function block is used to control an external actuator. Further information on this topic can be found
under FAQ #9 [} 329].

 Inputs
VAR_INPUT
    Enable:             BOOL;
    Enable_Positive:    BOOL;
    Enable_Negative:    BOOL;
    BufferMode:         MC_BufferMode_BkPlcMc:=Aborting_BkPlcMc;  //from V3.0.8
END_VAR

Name
Enable

Type
BOOL

Enable_Positive

BOOL

Enable_Negative

BOOL

BufferMode

MC_BufferMode_BkPlcMc

Description
A TRUE at this input activates an external actuator
of an axis.
A TRUE at this input activates the directional
enable of an external actuator of an axis for
movements in a positive direction.
A TRUE at this input activates the directional
enable of an external actuator of an axis for
movements in a negative direction.
Reserved. This input is provided in preparation for
a future build. It should currently either not be
assigned or assigned the constant
Aborting_BkPlcMc. (from V3.0.8)

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

 Outputs

VAR_OUTPUT
    Busy:       BOOL;
    Status:     BOOL;

Description
Here, the address of a variable of type
AXIS_REF_BkPlcMc [} 86] should be transferred.

TF5810

Version: 1.8.3

27

MC_Power_BkPlcMcEnable  BOOLEnable_Positive  BOOLEnable_Negative  BOOLBufferMode  MC_BufferMode_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  StatusBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

    Error:      BOOL;
    ErrorID:    UDINT;
END_VAR

Name
Busy
State
Error
ErrorID

Type
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
Readiness for operation is indicated here.
The occurrence of an error is indicated here.
An encoded error message is provided here.

Behavior of the function block

This function block is used to control external actuators. These can be modules for valve control (the valve's
onboard output stage or control cabinet assembly), frequency inverters or servo drives. These devices
usually require a digital signal to enable the output of energy through a power stage. Depending on the
design of the device, it is also possible for the "positive" and "negative" movement directions to be
individually activated.

The function block's input signals are passed on through the interface to the peripheral device. Enable also
activates error monitoring.

The function block investigates the axis interface that has been passed to it every time it is called. A number
of problems can be detected and reported during this process:

• If the value iTcMc_DriveAx2000_XXXXX is set under nDrive_Type in pStAxParams, the following

procedure is applied:

◦ If one of the pointers pStDeviceOutput or pStDeviceInput in AXIS_REF_BkPlcMc [} 86] is not

initialized, the function block responds with Error and ErrorID:=dwTcHydErrCdPtrPlcDriveIn or
dwTcHydErrCdPtrPlcDriveOut. Status is then FALSE.

◦ If an error is detected in the communication with the AX device or an error message occurs in the
pStDeviceInput interface of the AX device, the function block responds with Error and an ErrorID,
which is defined in the global constants [} 344] of the library. Status is then FALSE, and the axis
is set to an error state with the axis error dwTcHydErrCdDriveNotReady.

◦ Otherwise, the value of Enable is returned as the Status.

• If the value iTcMc_DriveKL2531 or iTcMc_DriveKL2541 is set under nDrive_Type in pStAxParams, the

following procedure is applied:

◦ The pointers pStDeviceOutput and pStDeviceInput in AXIS_REF_BkPlcMc [} 86] are checked. If

these pointers have not been initialized, the function block responds with Error and
ErrorID:=dwTcHydErrCdPtrPlcDriveIn or dwTcHydErrCdPtrPlcDriveOut. Status is then FALSE.

◦ If an error is detected in the communication with the I/O terminal or an error message of the

terminal occurs in the pStDeviceInput interface, the function block responds with Error and an
ErrorID, which is defined in the global constants [} 344] of the library. Status is then FALSE, and
the axis is set to an error state with the axis error dwTcHydErrCdDriveNotReady.

◦ Enable is used to activate the terminal output stage through a bit in

pStDeviceOutput.bTerminalCtrl. The ready signal in bTerminalCtrl.bTerminalState is returned as
Status.

◦ If the drive interface is operating without error, the value of Enable_Positive is entered with the

mask dwTcHydDcDwFdPosEna in the nDeCtrlDWord of pStAxRtData.

◦ If the drive interface is operating without error, the value of Enable_Negative is entered with the

mask dwTcHydDcDwFdNegEna in the nDeCtrlDWord of pStAxRtData.

• Otherwise the pointers pStDeviceInput and pStDeviceOutput in AXIS_REF_BkPlcMc [} 86] are checked.

If these pointers have not been initialized, the function block responds with Error and
ErrorID:=dwTcHydErrCdPtrPlcDriveIn or dwTcHydErrCdPtrPlcDriveOut. Status is then FALSE.

◦ Otherwise, the value of bPowerOk from pStDeviceInput is returned as the Status.

• If the drive interface is operating without error, the value of Enable is entered with the mask

dwTcHydDcDwCtrlEnable in the nDeCtrlDWord of pStAxRtData.

• If the drive interface is operating without error, the value of Enable_Positive is entered with the mask

dwTcHydDcDwFdPosEna in the nDeCtrlDWord of pStAxRtData.

28

Version: 1.8.3

TF5810

• If the drive interface is operating without error, the value of Enable_Negative is entered with the mask

dwTcHydDcDwFdNegEna in the nDeCtrlDWord of pStAxRtData.

This function block requires no time for executing its tasks. The output Busy will never assume the
value TRUE and only exists for compatibility reasons.

PLCopen Motion Control

4.1.2

MC_ReadActualPosition_BkPlcMc

Available from version 3.0

The function block determines the current position of an axis.

 Inputs
VAR_INPUT
    Enable:     BOOL;
END_VAR

Name
Enable

Type
BOOL

Description
Updating of the position value is initiated by a rising edge at this input.

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

 Outputs

VAR_OUTPUT
    Busy:       BOOL;
    Valid:      BOOL;
    Error:      BOOL;
    ErrorID:    UDINT;
    Position:   LREAL;
END_VAR

Description
Here, the address of a variable of type AXIS_REF_BkPlcMc should be
transferred.

Name
Busy
Valid
Error
ErrorID
Position

Type
BOOL
BOOL
BOOL
UDINT
LREAL

Description
Indicates that a command is being processed.
Successful determination of the actual position is indicated here.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.
[mm] The actual position.

TF5810

Version: 1.8.3

29

MC_ReadActualPosition_BkPlcMcEnable  BOOL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  ValidBOOL  ErrorUDINT  ErrorIDLREAL  PositionPLCopen Motion Control

Behavior of the function block

On a rising edge at Enable the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If the axis is in an error state and the cause is an encoder problem, the response is Error and

ErrorID:=error code of the encoder.

The actual position is determined and Valid is reported if these checks can be carried out without problems.

A falling edge at Enable clears all the pending output signals.

This function block requires no time for executing its tasks. The output Busy will never assume the
value TRUE and only exists for compatibility reasons.

4.1.3

MC_ReadActualTorque_BkPlcMc

Available from version 3.0

The function block determines the current actual force or actual pressure of an axis.

 Inputs
VAR_INPUT
    Enable:     BOOL;
END_VAR

Name
Enable

Type
BOOL

Description
A rising edge at this input triggers an update of the actual value.

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Valid:      BOOL;
    Busy:       BOOL;
    Error:      BOOL;
    ErrorID:    UDINT;
    Torque:     LREAL;
END_VAR

30

Version: 1.8.3

TF5810

MC_ReadActualTorque_BkPlcMcEnable  BOOL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ValidBOOL  BusyBOOL  ErrorUDINT  ErrorIDLREAL  TorquePLCopen Motion Control

Name
Valid
Busy
Error
ErrorID
Torque

Type
BOOL
BOOL
BOOL
UDINT
LREAL

Description
This indicates successful determination of the actual value.
This output is TRUE while the command is being processed.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.
The actual force or actual pressure.

Behavior of the function block

On a rising edge at Enable the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If the axis is in an error state and the cause is an encoder problem, the response is Error and

ErrorID:=error code of the encoder.

If these checks were completed without problem, the actual force or the actual pressure is determined, and
Valid is reported.

A falling edge at Enable clears all the pending output signals.

This function block requires no time for executing its tasks. The output Busy will never assume the
value TRUE and only exists for compatibility reasons.

4.1.4

MC_ReadActualVelocity_BkPlcMc

Available from version 3.0

The function block determines the current velocity of an axis.

 Inputs
VAR_INPUT
    Enable:     BOOL;
END_VAR

Name
Enable

Type
BOOL

Description
A rising edge at this input triggers an update of the velocity value.

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

TF5810

Version: 1.8.3

31

MC_ReadActualVelocity_BkPlcMcEnable  BOOL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ValidBOOL  BusyBOOL  ErrorUDINT  ErrorIDLREAL  VelocityPLCopen Motion Control

 Outputs

VAR_OUTPUT
    Valid:      BOOL;
    Busy:       BOOL;
    Error:      BOOL;
    ErrorID:    UDINT;
    Velocity:   LREAL;
END_VAR

Name
Valid
Busy
Error
ErrorID
Velocity

Type
BOOL
BOOL
BOOL
UDINT
LREAL

Description
This indicates successful determination of the velocity.
This output is TRUE while the command is being processed.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.
[mm/s] The actual velocity.

Behavior of the function block

On a rising edge at Enable the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If the axis is in an error state and the cause is an encoder problem, the response is Error and

ErrorID:=error code of the encoder.

The velocity is determined and reported with Valid if these checks can be carried out without problems.

A falling edge at Enable clears all the pending output signals.

This function block requires no time for executing its tasks. The output Busy will never assume the
value TRUE and only exists for compatibility reasons.

4.1.5

MC_ReadAxisError_BkPlcMc

Available from version 3.0

This function block determines the current error code of an axis.

 Inputs
VAR_INPUT
    Enable:     BOOL;
END_VAR

Name
Enable

Type
BOOL

Description
TRUE at this input triggers an update of the error code.

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

32

Version: 1.8.3

TF5810

MC_ReadAxisError_BkPlcMcEnable  BOOL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  ValidBOOL  ErrorUDINT  ErrorIDUDINT  AxisErrorIDName
Axis

Type
AXIS_RE
F_BkPlc
Mc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should be
transferred.

PLCopen Motion Control

 Outputs

VAR_OUTPUT
    Busy:       BOOL;
    Valid:      BOOL;
    Error:      BOOL;
    ErrorID:    UDINT;
    AxisErrorID:UDINT;
END_VAR

Name
Busy
Valid
Error

Type
BOOL
BOOL
BOOL

ErrorID

UDINT

Description
Indicates that a command is being processed.
Successful determination of the actual position is indicated here.
Indicates TRUE, if the function block was unable to execute the required
function.
Provides a coded cause of error, if the function block was unable to execute the
required function.

AxisErrorID

UDINT

Provides the current error code [} 339] of the axis.

Behavior of the function block

The function block checks the axis interface that has been passed to it if TRUE is asserted at Enable. The
current error code is reported as AxisErrorID. If Enable is FALSE, the function block cancels all pending
output signals.

This function block requires no time and no preconditions for executing its tasks. The outputs Error
and Busy will never assume the value TRUE and only exist for compatibility reasons.

4.1.6

MC_ReadBoolParameter_BkPlcMc

Available from version 3.0

This function block reads the boolean parameters of an axis. The function block MC_ReadParameter_BkPlcMc
[} 36] is available for non-boolean parameters.

 Inputs
VAR_INPUT
    Enable:             BOOL;
    ParameterNumber:    INT;
END_VAR

TF5810

Version: 1.8.3

33

MC_ReadBoolParameter_BkPlcMcEnable  BOOLParameterNumber  INT↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  ValidBOOL  ErrorUDINT  ErrorIDBOOL  ValuePLCopen Motion Control

Name
Enable
ParameterNumber

Type
BOOL
INT

Description
A reading process is initiated by a rising edge at this input.
This code number specifies the parameter that is to be read. Only named
constants from E_TcMCParameter [} 104] should be used.

 Inputs/outputs

VAR_IN_OUT
    Axis:               AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:               BOOL;
    Valid:              BOOL;
    Error:              BOOL;
    ErrorID:            UDINT;
    Value:              BOOL;
END_VAR

Name
Busy
Valid
Error
ErrorID
Value

Type
BOOL
BOOL
BOOL
UDINT
BOOL

Description
Indicates that a command is being processed.
Successful execution of the reading process is indicated here.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.
The value of the parameter is made available here.

Behavior of the function block

On a rising edge at Enable the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If an unsupported value is given to ParameterNumber the system responds with Error and

ErrorID:=dwTcHydErrCdNotSupport.

The desired parameter value is made available at Value, and Done is asserted if these checks can be
carried out without problems.

A falling edge at Enable clears all the pending output signals.

This function block requires no time for executing its tasks. The output Busy will never assume the
value TRUE and only exists for compatibility reasons.

4.1.7

MC_ReadDigitalOutput_BkPlcMc

34

Version: 1.8.3

TF5810

MC_ReadDigitalOutput_BkPlcMcEnable  BOOLOutputNumber  INT↔Output  Reference To OUTPUT_REF_BkPlcMcBOOL  ValidBOOL  BusyBOOL  ErrorUDINT  ErrorIDBOOL  ValuePLCopen Motion Control

Available from version 3.0

The function block determines the current state of a digital output of a cam controller.

 Inputs
VAR_INPUT
    Enable:         BOOL;
    OutputNumber:   INT;
END_VAR

Name
Enable
OutputNumber

Type
BOOL
INT

Description
A rising edge at this input triggers an update of the state.
The number of the output to be determined.

 Inputs/outputs

VAR_IN_OUT
    Output:     OUTPUT_REF_BkPlcMc;
END_VAR

Name
Output

Type
OUTPUT_REF_BkPlcMc

 Outputs

VAR_OUTPUT
    Valid:      BOOL;
    Busy:       BOOL;
    Error:      BOOL;
    ErrorID:    UDINT;
    Value:      BOOL;
END_VAR

Description

Here, the address of a variable of type OUTPUT_REF_BkPlcMc
[} 126] should be transferred.

Name
Valid
Busy
Error
ErrorID
Value

Type
BOOL
BOOL
BOOL
UDINT
BOOL

Description
This indicates successful determination of the state.
This output is TRUE while the command is being processed.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.
The state of the digital output.

Behavior of the function block

If Enable is TRUE, the function block checks the transferred parameters. During this process, a problem
may be detected and reported:

• If the value of OutputNumber is not within the permissible range [0..31], the response is Error with

ErrorID:=dwTcHydErrCdIllegalOutputNumber.

If these checks were carried out without problems, the state of the digital output is determined, and Valid is
reported.

A falling edge at Enable clears all the pending output signals.

TF5810

Version: 1.8.3

35

PLCopen Motion Control

4.1.8

MC_ReadParameter_BkPlcMc

Available from version 3.0

This function block reads the non-boolean parameters of an axis. The function block
MC_ReadBoolParameter_BkPlcMc [} 33] is available for boolean parameters.

 Inputs
VAR_INPUT
    Enable:             BOOL;
    ParameterNumber:    INT;
END_VAR

Name
Enable
ParameterNumber

Type
BOOL
INT

Description
A reading process is initiated by a rising edge at this input.
This code number specifies the parameter that is to be read. Only named
constants from E_TcMCParameter [} 104] should be used.

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc
[} 86] should be transferred.

 Outputs

VAR_OUTPUT
    Busy:       BOOL;
    Valid:      BOOL;
    Error:      BOOL;
    ErrorID:    UDINT;
    Value:      LREAL;
END_VAR

Name
Busy
Valid
Error
ErrorID
Value

Type
BOOL
BOOL
BOOL
UDINT
LREAL

Description
Indicates that a command is being processed.
Successful execution of the reading process is indicated here.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.
The value of the parameter is made available here.

Behavior of the function block

On a rising edge at Enable the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

36

Version: 1.8.3

TF5810

MC_ReadParameter_BkPlcMcEnable  BOOLParameterNumber  INT↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  ValidBOOL  ErrorUDINT  ErrorIDLREAL  ValuePLCopen Motion Control

• If an unsupported value is given to ParameterNumber the system responds with Error and

ErrorID:=dwTcHydErrCdNotSupport.

The desired parameter value is made available at Value, and Done is asserted if these checks can be
carried out without problems.

A falling edge at Enable clears all the pending output signals.

This function block requires no time for executing its tasks. The output Busy will never assume the
value TRUE and only exists for compatibility reasons.

4.1.9

MC_ReadStatus_BkPlcMc

Available from version 3.0

The function block determines the current state of an axis.

 Inputs
VAR_INPUT
    Enable:              BOOL;
END_VAR

Name
Enable

Type
BOOL

Description
A TRUE state at this input triggers an update of the function block.

 Inputs/outputs

VAR_IN_OUT
    Axis:               AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:               BOOL;
    Valid:              BOOL;

TF5810

Version: 1.8.3

37

MC_ReadStatus_BkPlcMcEnable  BOOL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  ValidBOOL  ErrorUDINT  ErrorIDBOOL  ErrorstopBOOL  DisabledBOOL  StoppingBOOL  StandStillBOOL  DiscreteMotionBOOL  ContinuousMotionBOOL  SynchronizedMotionBOOL  HomingBOOL  ConstantVelocityBOOL  AcceleratingBOOL  DeceleratingPLCopen Motion Control

    Error:              BOOL;
    ErrorID:            UDINT;
    Errorstop:          BOOL;
    Disabled:           BOOL;
    Stopping:           BOOL;
    StandStill:         BOOL;
    DiscreteMotion:     BOOL;
    ContinousMotion:    BOOL;
    SynchronizedMotion: BOOL;
    Homing:             BOOL;
    ConstantVelocity:   BOOL;
    Accelerating:       BOOL;
    Decelerating:       BOOL;
END_VAR

Name
Busy
Valid
Error

ErrorID

Errorstop

Disabled

Stopping

StandStill

DiscreteMotion

Type
BOOL
BOOL
BOOL

UDINT

BOOL

BOOL

BOOL

BOOL

BOOL

ContinousMotion

BOOL

SynchronizedMotion

BOOL

Homing
ConstantVelocity

Accelerating

BOOL
BOOL

BOOL

Decelerating

BOOL

Behavior of the function block

Description
Indicates that a command is being processed.
Successful determination of the actual position is indicated here.
This output reports any problems relating to the function of the function
block.
Provides a coded cause of error, if the function block was unable to
execute the required function.
This signal indicates that the axis associated with an error has been
placed in a state in which it is not able to operate. This state can only
be cleared by activating either a MC_Reset_BkPlcMc [} 39] or a
MC_ResetAndStop_BkPlcMc [} 40] function block.
This signal indicates whether the axis is enabled or disabled by its
MC_Power_BkPlcMc [} 27] function block.
This signal indicates that an active movement of the axis is being
stopped by a MC_Stop_BkPlcMc [} 82] or by a
MC_ResetAndStop_BkPlcMc [} 40] function block. This signal is
cleared as soon as the axis is stationary.
This signal indicates that the axis is neither in a fault state nor is it
active.
This signal indicates that the axis is executing an autonomous
movement (not resulting from a coupling) with a defined target.
This signal indicates that the axis is executing an autonomous
movement (not resulting from a coupling) with a defined velocity but not
with a specified target.
This signal indicates that the axis is being controlled by a gear
coupling.
This signal indicates that the axis is executing a homing.
This signal indicates that the axis is being moved with constant
velocity.
This signal indicates that the velocity of an axis is reaching a specified
value.

This does not always mean that the velocity is increasing: when
an axis that is already in movement is started, it can happen that
the axis accelerates in the direction opposite the current sense of
the velocity in order to achieve a specified velocity in the other
direction. From the point of view of the original movement this is a
deceleration, although from the point of the current (new)
movement it is still an acceleration.
This signal indicates that the axis is reducing its velocity in order to
continue a movement with a velocity lower than the current velocity, or
in order to end it.

If Enable is TRUE, the function block checks the transferred axis interface and decodes the internal state
information. A FALSE state at Enable clears all pending output signals.

38

Version: 1.8.3

TF5810

PLCopen Motion Control

This function block requires no time and no preconditions for executing its tasks. The outputs Error
and Busy will never assume the value TRUE and only exist for compatibility reasons.

Observe outputs

The outputs Error and ErrorID indicate the state of the function block, not that of the axis.

To read the current error code of the axis a MC_ReadAxisError_BkPlcMc() [} 32] function block must be used.

4.1.10

MC_Reset_BkPlcMc

Available from version 3.0

The function block eliminates an error state and puts the axis in an operational state.

 Inputs
VAR_INPUT
    Execute:    BOOL;
END_VAR

Name
Execute

Type
BOOL

Description
An axis reset is initiated by a rising edge at this input.

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:       BOOL;
    Done:       BOOL;
    Error:      BOOL;
    ErrorID:    UDINT;
END_VAR

Name
Busy
Done
Error
ErrorID

Type
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
Successful execution of the axis reset is indicated here.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

TF5810

Version: 1.8.3

39

MC_Reset_BkPlcMcExecute  BOOL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

Behavior of the function block

A rising edge at Execute triggers an axis reset. This puts the axis in an operational state, as far as possible,
and Done is reported. If this is not possible, the system responds with Error and ErrorID:= the ErrorCode of
the axis.

A falling edge at Execute clears all the pending output signals.

In some drive types, signal exchange with an external device is required, in order to rectify certain
errors. During the time required for this, the function block is unable to report a final result (Done or
Error). Instead, Busy is used to indicate that the function is in progress.

4.1.11

MC_ResetAndStop_BkPlcMc

Available from version 3.0

The function block puts a faulty axis in an operational state. If the axis is processing a travel command, this
is aborted, and the associated required stop operation is monitored.

 Inputs
VAR_INPUT
    Execute:        BOOL;
    Deceleration:   LREAL;  //from V3.0.5
    Jerk:           LREAL;  //from V3.0.5
    RampTime:       LREAL;  //from V3.0.5
    BufferMode:     MC_BufferMode_BkPlcMc:=Aborting_BkPlcMc;  //from V3.0.8
END_VAR

Name
Execute

Type
BOOL

Deceleration
Jerk
RampTime
BufferMode

LREAL
LREAL
LREAL
MC_BufferMode_BkPlcMc

Description
A rising edge at this input triggers an axis reset and a
stop operation.
[mm/s2] The deceleration to be applied.
[mm/s3] The jerk to be applied.
[s] The required stopping time.
Reserved. This input is provided in preparation for a
future build. It should currently either not be assigned or
assigned the constant Aborting_BkPlcMc. (from V3.0.8)

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

40

Version: 1.8.3

TF5810

MC_ResetAndStop_BkPlcMcExecute  BOOLDeceleration  LREALJerk  LREALRampTime  LREALBufferMode  MC_BufferMode_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  ErrorUDINT  ErrorIDBOOL  ActivePLCopen Motion Control

 Outputs

VAR_OUTPUT
    Busy:       BOOL;
    Done:       BOOL;
    Error:      BOOL;
    ErrorID:    UDINT;
END_VAR

Name
Busy
Done
Error
ErrorID

Type
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
Successful execution of the axis reset is indicated here.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If it is not possible to successfully clear an existing error state for an axis through a reset operation, the

system responds with Error and ErrorID:= the ErrorCode for the axis.

• If the axis is placed into an error state in the course of a stop operation that may have been necessary,

the system responds with Error and ErrorID:= the ErrorCode for the axis.

Successful completion of both operations is reported with Done. The axis is then without error and
stationary.

A falling edge at Execute clears all the pending output signals.

If the axis is executing a motion, it is decelerated until it stops. In some drive types, signal exchange
with an external device is required, in order to rectify certain errors. During the time required for this,
the function block is unable to report a final result (Done or Error). Instead, Busy is used to indicate
that the function is in progress.

4.1.12

MC_SetOverride_BkPlcMc

Available from version 3.0

The function block sets the override of an axis.

This function block only takes effect if the profile type iTcMc_ProfileCtrlBased is used.

 Inputs
VAR_INPUT
    Enable:     BOOL;
    VelFactor:  LREAL;
END_VAR

TF5810

Version: 1.8.3

41

MC_SetOverride_BkPlcMcEnable  BOOLVelFactor  LREAL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  EnabledBOOL  BusyBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

Name
Enable
VelFactor

Type
BOOL
LREAL

Description
An active state at this input sets the override of the axis.
[1] The new override of the axis.

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Enabled:    BOOL;
    Busy:       BOOL;
    Error:      BOOL;
    ErrorID:    UDINT;
END_VAR

Name
Enabled
Busy
Error
ErrorID

Type
BOOL
BOOL
BOOL
UDINT

Description
This indicates the active state of the function block.
This output is TRUE while the command is being processed.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

If the Enable state is active, the value transferred as VelOverride is limited to the range 0.0 to 1.0 and
entered in Axis.pStAxParams^.fOverride. Enabled is set to TRUE.

A falling edge at Enable clears all outputs.

42

Version: 1.8.3

TF5810

PLCopen Motion Control

All velocity changes caused by an override modification are limited according to the maximum permitted
accelerations and decelerations.

In order to ensure reproducible behavior during the target approach, the override only reduces the
travel speed to pStAxParams.fCreepSpeed. Therefore, it is not possible to stop the axis movement
through an override of 0.0.

4.1.13

MC_SetPosition_BkPlcMc

Available from version 3.0

The function block sets the actual position of an axis.

 Inputs
VAR_INPUT
    Execute:    BOOL;
    Position:   LREAL;
    Mode:       BOOL;
END_VAR

TF5810

Version: 1.8.3

43

MC_SetPosition_BkPlcMcExecute  BOOLPosition  LREALMode  BOOL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  DoneBOOL  BusyBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

Name
Execute
Position
Mode

Type
BOOL
LREAL
BOOL

Description
A rising edge at this input sets the actual position of the axis.
[mm] The new actual position of the axis.
This parameter specifies the operating mode. If Mode = TRUE, the actual position is
changed by Position, if Mode = FALSE, the actual position is set to Position.

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Done:       BOOL;
    Busy:       BOOL;
    Error:      BOOL;
    ErrorID:    UDINT;
END_VAR

Name
Done
Busy
Error
ErrorID

Type
BOOL
BOOL
BOOL
UDINT

Description
This indicates successful processing of the command.
This output is TRUE while the command is being processed.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• Depending on the encoder type specified in Axis.pStAxParams^.nEnc_Type, either

ST_TcHydAxRtData.fEnc_RefShift or ST_TcHydAxParam.fEnc_ZeroShift is updated such that the
actual position of the axis assumes the required value. If the encoder type is unknown or the encoder
does not permit the actual value to be set, the system responds with Error and
ErrorID:=dwTcHydErrCdEncType.

• If ST_TcHydAxParam.fEnc_ZeroShift changes recognizable during this process, AXIS_REF_BkPlcMc

[} 86].ST_TcHydAxRtData [} 141].bParamsUnsave is set.

This function block may cause the actual position and/or the target position of the currently
processed motion to be moved after an active software limit switch. This is not monitored by the
function block.

If these checks could be performed without problem, all other affected elements in ST_TcHydAxRtData are
automatically updated. This function block can therefore also be activated for axes, which perform an active
motion. The successful execution of the function is indicated with Done. A falling edge at Execute clears all
the pending output signals.

44

Version: 1.8.3

TF5810

4.1.14

MC_SetReferenceFlag_BkPlcMc

PLCopen Motion Control

Available from version 3.0

(Function is not defined by PLCopen) The function block defines the referencing flag of the axis.

 Inputs
VAR_INPUT
    Execute:        BOOL;
    ReferenceFlag:  BOOL;
END_VAR

Name
Execute
ReferenceFlag

Type
BOOL
BOOL

Description
A rising edge at this input sets the referencing flag of the axis.
The new state of the referencing flag of the axis.

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Done:       BOOL;
    Busy:       BOOL;
    Error:      BOOL;
    ErrorID:    UDINT;
END_VAR

Name
Done
Busy
Error
ErrorID

Type
BOOL
BOOL
BOOL
UDINT

Description
This indicates successful processing of the command.
This output is TRUE while the command is being processed.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

A rising edge at Execute causes the referencing flag in ST_TcHydAxRtData.nStateDWord [} 338] to be
updated. To this end, the respective bit is cleared or set with dwTcHydNsDwReferenced, depending on
ReferenceFlag. The successful execution of the function is indicated with Done. A falling edge at Execute
clears all the pending output signals.

TF5810

Version: 1.8.3

45

MC_SetReferenceFlag_BkPlcMcExecute  BOOLReferenceFlag  BOOL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  DoneBOOL  BusyBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

4.1.15

MC_WriteBoolParameter_BkPlcMc

Available from version 3.0

This function block writes the boolean parameters of an axis. The function block
MC_WriteParameter_BkPlcMc [} 48] is available for non-boolean parameters.

 Inputs
VAR_INPUT
    Execute:            BOOL;
    ParameterNumber:    INT;
    Value:              BOOL;
END_VAR

Name
Execute
ParameterNumber

Value

Type
BOOL
INT

BOOL

Description
A write process is initiated by a rising edge at this input.
This code number specifies the parameter that is to be read. Only named
constants from E_TcMCParameter [} 104] should be used.
The value of the parameter is to be provided here.

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description
Here, the address of a variable of type AXIS_REF_BkPlcMc should be
transferred.

 Outputs

VAR_OUTPUT
    Busy:       BOOL;
    Done:       BOOL;
    Error:      BOOL;
    ErrorID:    UDINT;
END_VAR

Name
Busy
Done
Error
ErrorID

Type
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
Successful execution of the writing process is indicated here.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Enable the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If an unsupported value is given to ParameterNumber the system responds with Error and

ErrorID:=dwTcHydErrCdNotSupport.

46

Version: 1.8.3

TF5810

MC_WriteBoolParameter_BkPlcMcExecute  BOOLParameterNumber  INTValue  BOOL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

If these checks could be performed without problems Value is entered into the desired parameter value and
Done is reported. If the parameter is changed in the process AXIS_REF_BkPlcMc [} 86].ST_TcHydAxRtData
[} 141].bParamsUnsave is set.

A falling edge at Enable clears all the pending output signals.

This function block requires no time for executing its tasks. The output Busy will never assume the
value TRUE and only exists for compatibility reasons.

4.1.16

MC_WriteDigitalOutput_BkPlcMc

Available from version 3.0

The function block determines the state of a digital output of a cam controller.

 Inputs
VAR_INPUT
    Execute:        BOOL;
    OutputNumber:   INT;
    Value:          BOOL;
END_VAR

Name
Execute
OutputNumber
Value

Type
BOOL
INT
BOOL

Description
A rising edge at this input triggers an update of the state.
The number of the output to be determined.
The state of the digital output.

 Inputs/outputs

VAR_IN_OUT
    Output:     OUTPUT_REF_BkPlcMc;
END_VAR

Name
Output

Type
OUTPUT_REF_BkPlcMc

 Outputs

VAR_OUTPUT
    Done:       BOOL;
    Busy:       BOOL;
    Error:      BOOL;
    ErrorID:    UDINT;
END_VAR

Description
Here, the address of a variable of type OUTPUT_REF_BkPlcMc
should be transferred.

TF5810

Version: 1.8.3

47

MC_WriteDigitalOutput_BkPlcMcExecute  BOOLOutputNumber  INTValue  BOOL↔Output  Reference To OUTPUT_REF_BkPlcMcBOOL  DoneBOOL  BusyBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

Name
Done
Busy
Error
ErrorID

Type
BOOL
BOOL
BOOL
UDINT

Description
This indicates successful determination of the state.
This output is TRUE while the command is being processed.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

A rising edge at Execute causes the function block to check the transferred parameters. During this process,
a problem may be detected and reported:

• If the value of OutputNumber is not within the permissible range [0..31], the response is Error with

ErrorID:=dwTcHydErrCdIllegalOutputNumber.

If these checks could be performed without problems, the state of the digital output is defined according to
the value of Value, and Done is reported.

A falling edge at Execute clears all the pending output signals.

4.1.17

MC_WriteParameter_BkPlcMc

Available from version 3.0

This function block writes the non-boolean parameters of an axis. The function block
MC_WriteBoolParameter_BkPlcMc [} 46] is available for boolean parameters.

 Inputs
VAR_INPUT
    Enable:             BOOL;
    ParameterNumber:    INT;
    Value:              LREAL;
END_VAR

Name
Enable
ParameterNumber

Value

Type
BOOL
INT

LREAL

Description
A write process is initiated by a rising edge at this input.
This code number specifies the parameter that is to be read. Only named
constants from E_TcMCParameter [} 104] should be used.
The value of the parameter is to be provided here.

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

48

Version: 1.8.3

TF5810

MC_WriteParameter_BkPlcMcExecute  BOOLParameterNumber  INTValue  LREAL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

 Outputs

VAR_OUTPUT
    Busy:       BOOL;
    Done:       BOOL;
    Error:      BOOL;
    ErrorID:    UDINT;
END_VAR

Name
Busy
Done
Error
ErrorID

Type
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
Successful execution of the writing process is indicated here.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Enable the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If an unsupported value is given to ParameterNumber the system responds with Error and

ErrorID:=dwTcHydErrCdNotSupport.

If these checks could be performed without problems Value is entered into the desired parameter value and
Done is reported. If the parameter is changed recognizably in the process AXIS_REF_BkPlcMc
[} 86].ST_TcHydAxRtData [} 141].bParamsUnsave is set.

A falling edge at Enable clears all the pending output signals.

This function block requires no time for executing its tasks. The output Busy will never assume the
value TRUE and only exists for compatibility reasons.

4.2

Motion

4.2.1

MC_CamIn_BkPlcMc

Available from version 3.0

The function block starts and monitors a cam plate coupling between two axes. To release the coupling, an
MC_CamOut_BkPlcMc [} 51] function block should be used.

TF5810

Version: 1.8.3

49

MC_CamIn_BkPlcMcExecute  BOOLMasterOffset  LREALSlaveOffset  LREALMasterScaling  LREALSlaveScaling  LREALStartMode  MC_StartMode_BkPlcMcCamTableId  MC_CAM_ID_BkPlcMcBufferMode  MC_BufferMode_BkPlcMc↔Master  Reference To AXIS_REF_BkPlcMc↔Slave  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  InSyncBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDBOOL  EndOfProfilePLCopen Motion Control

 Inputs
VAR_INPUT
    Execute:        BOOL;
    MasterOffset:   LREAL:=0.0;
    SlaveOffset:    LREAL:=0.0;
    MasterScaling:  LREAL:=0.0;
    SlaveScaling:   LREAL:=0.0;
    StartMode:      MC_StartMode_BkPlcMc:=MC_StartMode_Absolute;
    CamTableId:     MC_CAM_ID_BkPlcMc;
    BufferMode:     MC_BufferMode_BkPlcMc:=Aborting_BkPlcMc;    //from V3.0.8
END_VAR

Name
Execute
MasterOffset

Type
BOOL
LREAL

SlaveOffset

LREAL

MasterScaling

LREAL

SlaveScaling

LREAL

StartMode

MC_StartMode_BkPlcMc

CamTableId

MC_CAM_ID_BkPlcMc

BufferMode

MC_BufferMode_BkPlcMc

 Inputs/outputs

VAR_IN_OUT
    Master:         AXIS_REF_BkPlcMc;
    Slave:          AXIS_REF_BkPlcMc;
END_VAR

Description
A rising edge at this input starts the coupling.
[mm, 1] This value is offset against with the actual
position of the master, before the resulting value is
looked up in the master column of the table.
[mm, 1] This value is offset against the slave position
from the table.
[mm, 1] This value is offset against with the actual
position of the master, before the resulting value is
looked up in the master column of the table.
[mm, 1] This value is offset against the slave position
from the table.

A value from MC_StartMode_BkPlcMc [} 125], which
specifies the behavior of the slave axis when the
coupling is activated.

Here, a variable of type MC_CAM_ID_BkPlcMc [} 122]
should be transferred, which was initialized by a
function block of type MC_CamTableSelect_BkPlcMc
[} 53].
Reserved. This input is provided in preparation for a
future build. It should currently either not be assigned
or assigned the constant Aborting_BkPlcMc. (from
V3.0.8)

Name
Master

Type
AXIS_REF_BkPlcMc

Slave

AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    InSync:         BOOL;
    CommandAborted: BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
    EndOfProfile:   BOOL;
END_VAR

50

Version: 1.8.3

TF5810

PLCopen Motion Control

Name
Busy
InSync

CommandAborted
Error
ErrorID
EndOfProfile

Type
BOOL
BOOL

BOOL
BOOL
UDINT
BOOL

Description
Indicates that a command is being processed.
This indicates the first successful synchronization of the axes. The signal
the remains active, even if the synchronization subsequently fails
temporarily or permanently.
This indicates abortion of the coupling.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.
This is indicates whether the master has reached the end of the defined
range.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If CamTableId.bValidated was not set by a function block of type MC_CamTableSelect_BkPlcMc, the

system responds with Error and ErrorID:=dwTcHydErrCdTblNoInit.

• If either the master or the slave are not in idle state, the system responds with Error and

ErrorID:=dwTcHydErrCdNotStartable.

• If the value MC_StartMode_RampIn is specified as StartMode, the function block responds with Error

and ErrorID:=dwTcHydErrCdNotSupport.

If these checks could be performed without problem, the coupling is initiated. Depending on StartMode, the
reference position for Slave is either set to 0.0 or to the current actual position of Slave. The axis is now in
state McState_Synchronizedmotion [} 103], and the function block starts calculating and monitoring the
coupling.

The set position and set velocity of Slave are calculated depending on the actual position and the set
velocity of the master and the table.

When the velocity required by the coupling is reached for the first time while the slave axis coupling is active,
this is indicated at output InGear. Since the coupling can currently only be activated at standstill, this is the
case immediately. If the slave axis is unable to follow the specifications for some reason while the coupling is
active, InGear remains unchanged.

If an error code occurs in the motion generator while the coupling is active, the system responds with Error
and ErrorID:=motion algorithm error code.

A falling edge at Execute neither aborts the calculation nor the monitoring of the coupling. This is only
brought about if the coupling is activated through an MC_CamOut_BkPlcMc function block or if an error
occurs. Only then are all pending output signals cleared.

This function block temporarily deals with setpoint generation. To indicate this, Busy is not only
TRUE up to the transition to synchronicity, but remains TRUE until the coupling is released.

Function block call

It is mandatory to call this function block cyclically when Busy is TRUE. Subsequently, the function
block should be called at least once with Execute:=FALSE.

4.2.2

MC_CamOut_BkPlcMc

TF5810

Version: 1.8.3

51

MC_CamOut_BkPlcMcExecute  BOOL↔Slave  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

Available from version 3.0

The function block releases a cam plate coupling between two axes, which was started through an
MC_CamIn_BkPlcMc [} 49] function block.

 Inputs
VAR_INPUT
    Execute:        BOOL;
ND_VAR

Name
Execute

Type
BOOL

Description
A rising edge at this input starts the coupling.

 Inputs/outputs

VAR_IN_OUT
    Slave:          AXIS_REF_BkPlcMc;
END_VAR

Name
Slave

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    Done:           BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
Busy
Done
Error
ErrorID

Type
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
This indicates successful processing of the command.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If the pointer pStAxParams in AXIS_REF_BkPlcMc [} 86] is not initialized, the system responds with

Error and ErrorID:=dwTcHydErrCdPtrPlcMc.

• If the pointer pStAxParams in AXIS_REF_BkPlcMc [} 86] is not initialized, the system responds with

Error and ErrorID:=dwTcHydErrCdPtrMcPlc.

• If the axis is not coupled, the function block responds with Done, without further checks or activities.

• If the current set velocity of the axis is smaller than the velocity specified by

pStAxParams.fCreepSpeed, the axis immediately assumes McState_Standstill and dissipates the
residual velocity. Done is indicated, and no further checks or activities take place.

If these checks could be performed without problem and Done is not already indicated for one of the reasons
mentioned, the motion controlled by the cam plate coupling is converted to a continuous motion with the
same velocity and direction, which is independent of the master. Done is indicated if this conversion was
executed successfully, otherwise the system responds with Error and ErrorID:=error code.

52

Version: 1.8.3

TF5810

This function block requires no time for executing its tasks. The output Busy will never assume the
value TRUE and only exists for compatibility reasons.

PLCopen Motion Control

4.2.3

MC_CamTableSelect_BkPlcMc

Available from version 3.0

The function block initializes a variable of type MC_CAM_ID_BkPlcMc [} 122], thereby preparing a cam plate
for the coupling of two axes.

 Inputs
VAR_INPUT
    Execute:            BOOL;
    Periodic:           BOOL;
    MasterAbsolute:     BOOL;
    SlaveAbsolute:      BOOL;
ND_VAR

Name
Execute
Periodic
MasterAbsolute
SlaveAbsolute

Type
BOOL
BOOL
BOOL
BOOL

Description
A rising edge at this input starts the command.
Not supported: FALSE is currently to be passed here.
Not supported: TRUE is currently to be passed here.
Not supported: TRUE is currently to be passed here.

 Inputs/outputs

VAR_IN_OUT
    Master:             AXIS_REF_BkPlcMc;
    Slave:              AXIS_REF_BkPlcMc;
    CamTable:           MC_CAM_REF_BkPlcMc;
END_VAR

Name
Master

Type
AXIS_REF_BkPlcMc

Slave

AXIS_REF_BkPlcMc

CamTable

MC_CAM_REF_BkPlcMc

 Outputs

VAR_OUTPUT
    Busy:               BOOL;
    Done:               BOOL;
    Error:              BOOL;

Description
Here, the address of a variable of type AXIS_REF_BkPlcMc
should be transferred.
Here, the address of a variable of type AXIS_REF_BkPlcMc
should be transferred.

A variable of type MC_CAM_REF_BkPlcMc [} 123] should be
transferred here.

TF5810

Version: 1.8.3

53

MC_CamTableSelect_BkPlcMcExecute  BOOLPeriodic  BOOLMasterAbsolute  BOOLSlaveAbsolute  BOOL↔Master  Reference To AXIS_REF_BkPlcMc↔Slave  Reference To AXIS_REF_BkPlcMc↔CamTable  Reference To MC_CAM_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  ErrorUDINT  ErrorIDMC_CAM_ID_BkPlcMc  CamTableIdPLCopen Motion Control

    ErrorID:            UDINT;
    CamTableId:         MC_CAM_ID_BkPlcMc;
END_VAR

Name
Busy
Done
Error
ErrorID

Type
BOOL
BOOL
BOOL
UDINT

CamTableId

MC_CAM_ID_BkPlcMc

Description
Indicates that a command is being processed.
This indicates successful initialization of CamTableId.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided
here.

Returns a variable of type MC_CAM_ID_BkPlcMc [} 122],
which can be passed on to a function block of type
MC_CamIn_BkPlcMc [} 49].

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If CamTable.pTable is not initialized the system responds with Error and

ErrorID:=dwTcHydErrCdPtrPlcMc.

• If CamTable.nLastIdx is not greater than CamTable.nFirstIdx the system responds with Error and

ErrorID:=dwTcHydErrCdTblEntryCount.

• If CamTable.nFirstIdx and CamTable.nLastIdx define a table with more than 100 rows the system

responds with Error and ErrorID:=dwTcHydErrCdTblLineCount.

• If MasterAbsolute or SlaveAbsolute are not set or Periodic is set, the system responds with Error

and ErrorID:=dwTcHydErrCdNotSupport.

If these checks could be performed without problem, CamTableId is initialized. The data from CamTable
and the input data of function block are used for this purpose. CamTableId is marked as valid and modified.
Done is used to report execution of the command.

A falling edge at Execute clears all the pending output signals.

This function block requires no time for executing its tasks. The output Busy will never assume the
value TRUE and only exists for compatibility reasons.

4.2.4

MC_DigitalCamSwitch_BkPlcMc

Available from version 3.0

The function block generates software cams depending on the position, direction of travel and velocity of an
axis.

54

Version: 1.8.3

TF5810

MC_DigitalCamSwitch_BkPlcMcEnable  BOOLEnableMask  DWORD↔Axis  Reference To AXIS_REF_BkPlcMc↔Switches  Reference To CAMSWITCH_REF_BkPlcMc↔Outputs  Reference To OUTPUT_REF_BkPlcMc↔TrackOptions  Reference To TRACK_REF_BkPlcMcBOOL  InOperationBOOL  BusyBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

 Inputs
VAR_INPUT
    Enable:         BOOL;
    EnableMask:     DWORD;
END_VAR

Name
Enable
EnableMask

Type
BOOL
DWORD

Description
This input controls all activities of the function block.
A mask with bits that specify the activation of the outputs in Outputs.

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
    Switches:       CAMSWITCH_REF_BkPlcMc;
    Outputs:        OUTPUT_REF_BkPlcMc;
    TrackOptions:   TRACK_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Switches

CAMSWITCH_REF_BkPlcMc

Outputs

OUTPUT_REF_BkPlcMc

TrackOptions

TRACK_REF_BkPlcMc

 Outputs

VAR_OUTPUT
    InOperation:    BOOL;
    Busy:           BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Description
Here, the address of a variable of type
AXIS_REF_BkPlcMc [} 86] should be transferred.

Here, an array of type CAMSWITCH_REF_BkPlcMc
[} 123] should be transferred.
Here, the address of a variable of type
OUTPUT_REF_BkPlcMc [} 126] should be transferred.

Here, an array of type TRACK_REF_BkPlcMc [} 126]
should be transferred.

Name
InOperation
Busy
Error
ErrorID

Type
BOOL
BOOL
BOOL
UDINT

Description
This indicates whether the function block is active.
This output is TRUE while the command is being processed.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

Cam signals (switches) are switched based on the actual position of an axis. The available options are
position-controlled (with start and end position) and time-controlled (with trigger position and duration). The
direction of travel of the axis can be taken into account.

The cam signals are assigned to tracks with parameter sable properties. The time response can be specified
through a switch-on and switch-off delay. Predictive signalling can be achieved through negative values. A
hysteresis enables suppression of undesirable signalling, if the axis is near a switching points and the actual
position is not entirely constant.

Example

CAMSWITCH_REF_BkPlcMc [} 123] used:

TF5810

Version: 1.8.3

55

PLCopen Motion Control

Parameter
TrackNumber
FirstOnPositio
n
LastOnPositio
n
AxisDirection
CamSwitchMo
de
Duration
.....

Switch[1]

Switch[2]

Switch[3]

Switch[4]

...

Switch[n]

1
2000.0

1
2500.0

1
-1000.0

2
3000.0

3000.0

3000.0

1000.0

1
0

2
0

0
0

0
1

1.35

TRACK_REF_BkPlcMc [} 126] used:

Parameter
OnCompensation
OffCompensation
Hysteresis

Track[1]

Track[2]

...

Track[n]

-0.125
0.250
0.0

0.0
0.0
0.0

Signal curves during axis motion from 0.0 to 5000.0 and back:

The following diagram shows the signal curves over the position. For positive direction of travel the signals
are shown normally (upwards), for negative direction of travel they are shown negative, i.e. 'downwards'. The
vertical cursor lines indicate the positions 1000 and 3000 mm.

56

Version: 1.8.3

TF5810

PLCopen Motion Control

4.2.5

MC_EmergencyStop_BkPlcMc

Available from version 3.0.5

The function block cancels a current axis motion and monitors the emergency stop operation.

 Inputs
VAR_INPUT
    Execute:        BOOL;
    RampTime:       LREAL;  //from V3.0.5
END_VAR

Name
Execute
RampTime

Type
BOOL
LREAL

Description
A rising edge at this input ends a movement being carried out by the axis.
[s] The required stopping time.

TF5810

Version: 1.8.3

57

MC_EmergencyStop_BkPlcMcExecute  BOOLRampTime  LREAL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  ErrorUDINT  ErrorIdBOOL  ActiveBOOL  CommandAbortedPLCopen Motion Control

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    Done:           BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
    Active:         BOOL;
    CommandAborted: BOOL;
END_VAR

Name
Busy
Done
Error
ErrorID
Active
CommandAborted

Type
BOOL
BOOL
BOOL
UDINT
BOOL
BOOL

Description
Indicates that a command is being processed.
This indicates successful processing of the operation.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.
Indicates that a command is being processed.
Indicates that processing of this command was aborted by another
command.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• The stop can only be executed if the axis is actively carrying out a movement. If it is stationary, the

function block immediately asserts the Done signal.

• If the axis is already in an error state, or if it is in the process of carrying out a stop operation, it

responds with Error and ErrorID:=dwTcHydErrCdNotReady.

• If the axis is in a state, in which it is controlled by a coupling with another axis or a comparable

mechanism, it responds with Error and ErrorID:=dwTcHydErrCdNotReady.

The Stop operation begins if these checks can be carried out without problems. RampTime is used to
calculate a deceleration, taking into account the reference velocity. MaxJerk is used if a jerk-limiting control
value generator is selected. If no value is specified for RampTime, which is recognizably greater than 0, the
axis parameter fEmergencyRamp is used.

An MC_Stop_BkPlcMc [} 82] function block is used internally for slowing down the axis. Once the control
value output is reduced to 0, all control or regulating voltage outputs are suppressed, as long as Execute is
set to TRUE.

58

Version: 1.8.3

TF5810

4.2.6

MC_FlyingGear_BkPlcMc

PLCopen Motion Control

The function block takes over the control of a flying gear coupling.

 Inputs
VAR_INPUT
    Enable:                 BOOL;
    Ratio:                  LREAL;
    MasterSyncPosition:     LREAL;
    SlaveSyncPosition:      LREAL;
    SlaveDesyncPosition:    LREAL;
    SlaveStopPosition:      LREAL;
    DefaultSlaveNegative:   BOOL;
END_VAR

Name
Enable

Ratio

Type
BOOL

LREAL

MasterSyncPosition

LREAL

SlaveSyncPosition

LREAL

SlaveDesyncPosition

LREAL

SlaveStopPosition

LREAL

DefaultSlaveNegative

BOOL

Description
This signal allows the function block to become active as soon as the
master's position passes the MasterSyncPosition in the designated
direction.
This parameter specifies the gear ratio between master and slave for
the fully coupled case.
This parameter indicates the position in the designated direction of
the master at which the coupling should have been fully established.
This parameter indicates the position in the designated direction of
the slave at which the coupling should have been fully established.
This parameter indicates the position in the designated direction of
the slave at which the release of the coupling should start.
This parameter indicates the position in the designated direction of
the slave at which the release of the coupling should have been
completed.
In some cases, the function block cannot determine the intended work
direction from the transferred parameters. Additional information is
then required from the application.

 Inputs/outputs

VAR_IN_OUT
    Master:                 AXIS_REF_BkPlcMc;
    Slave:                  AXIS_REF_BkPlcMc;
END_VAR

Name
Master

Type
AXIS_REF_BkPlcMc

Slave

AXIS_REF_BkPlcMc

Description
Here, the address of a variable of type AXIS_REF_BkPlcMc should
be transferred.
Here, the address of a variable of type AXIS_REF_BkPlcMc should
be transferred.

TF5810

Version: 1.8.3

59

MC_FlyingGear_BkPlcMcEnable  BOOLRatio  LREALMasterSyncPosition  LREALSlaveSyncPosition  LREALSlaveDesyncPosition  LREALSlaveStopPosition  LREALDefaultSlaveNegative  BOOLTurnPoint  BOOL↔Master  Reference To AXIS_REF_BkPlcMc↔Slave  Reference To AXIS_REF_BkPlcMcBOOL  StartSyncBOOL  InSyncBOOL  StopSyncBOOL  BusyBOOL  ActiveBOOL  ErrorUDINT  ErrorIdPLCopen Motion Control

 Outputs

VAR_OUTPUT
    StartSync:              BOOL;
    InSync:                 BOOL;
    Busy:                   BOOL;
    Active:                 BOOL;
    Error:                  BOOL;
    ErrorId:                BOOL;
END_VAR

Name
StartSync

InSync
Busy

Type
BOOL

BOOL
BOOL

Active

BOOL

Error

BOOL

ErrorId

BOOL

Description
This signal is TRUE if the master is within the section for the establishment of the
coupling.
This signal is TRUE if the coupling is fully active.
This signal goes TRUE if Enable is TRUE, master and slave are ready and not in
an error state and the transferred parameters are suitable.
This signal goes TRUE as soon as the master passes the MasterSyncPosition in
the designated direction. It goes FALSE when the SlaveStopPosition is reached in
the opposite direction of the slave.
This signal goes TRUE if there are unsuitable parameters on a rising edge at
Enable or if the master or slave is in an error state on a TRUE at Enable.
A numerically encoded indication of an error cause.

Behavior of the function block

The function block establishes a coupling between a master and a slave axis that is established and
released on the fly. The parameterization and enable can vary the behavior in wide areas. In combination
with positioning commands, a variety of motion sequences can be realized. Just a few samples are shown
here.

The scopes were created using the sample S106_FlyingGear. The number of the sample corresponds to
the selected nSequence.

NOTICE

During commissioning

When measuring the position of the master, the zero point should be shifted so that the actual position
represents comprehensible information about the situation in the machine.

For the slave, the zero shift should be chosen in such a way that the actual position, taking into account the
dimensions of the material, tools and other installations, matches that of the master, if the remaining gap has
just become 0. To avoid elastic deformation, no force should have been built up.

Sample #1

Here, master and slave move in the same direction. Both return to their starting position.

60

Version: 1.8.3

TF5810

PLCopen Motion Control

Preparation:

• The master was positioned at 800.0 mm.

• The slave was positioned at 500.0 mm.

• The location at which the slave begins with the establishment of the synchronization is thus defined.

• The final coupling factor (ratio) has been set to 1.0 for better understanding. In practical use, this factor

is often chosen slightly smaller in order to trigger a transition to a pressure or force control.

• The point for achieving synchronization has been set to 460.0 mm for the master (MasterSyncPosition)

and 450.0 mm for the slave (SlaveSyncPosition).

• The difference of 10.0 mm between master and slave represents the distance required for material,
tools and other installations and is chosen here in such a way that the representation illustrates the
behavior.

• The distance for synchronizing the slave is calculated from the starting position of the slave and the

SlaveSyncPosition and is in this case 500.0 - 450.0 => 50 mm.

• With a ratio of 1.0, the master has a distance to synchronize of 2.0 * 50 mm => 100 mm. So the

synchronization of the master will start at 460.0 mm+100 mm => 560 mm (MasterSyncPosition plus
distance). If the master is below this position when the coupling is enabled, an error is signaled.

• The point for leaving the synchronization has been set to 450.0 mm for the slave

(SlaveDesyncPosition).

• The point for the complete release of the coupling (SlaveStopPosition) has been set to 500.0 mm. The

intended distance for this is thus 500.0 mm – 450 mm => 50 mm.

• With a ratio of 1.0, the master will also travel a distance of 2.0 * 50 mm => 100 mm.

• Distances that the master travels before TM_1 and after TM_4 in the above scope do not concern the

slave.

• If the master changes its speed between TM_1 and TM_2 or between TM_3 and TM_4, the slave will

take this into account. Then its speed curve may be less clear than in the above scope.

Sample #2

Here, the master and slave move in opposite directions. Both return to their starting position.

TF5810

Version: 1.8.3

61

PLCopen Motion Control

• Here, the starting position is not 50 mm above the SlaveSyncPosition, but by the same amount below

it.

Sample #3

Here, master and slave move in the same direction. However, the slave only returns to its starting position by
means of an MC_MoveAbsolut_BkPlcMc().

• Here, the initial situation is the same as in the sample #1.

• However, the SlaveStopPosition is different here. The slave accordingly stops earlier.

62

Version: 1.8.3

TF5810

• The distance between SlaveDesyncPosition and SlaveStopPosition is 100 mm here. As a result, the

release of the coupling creates a different profile than the establishment of the coupling.

• Once the master has completed its movement, the function block is deactivated and the slave is driven

to the starting position with its own command.

PLCopen Motion Control

4.2.7

MC_GearIn_BkPlcMc

Available from version 3.0

The function block starts and monitors a coupling between two axes. To release the coupling, an
MC_GearOut_BkPlcMc [} 67] function block should be used.

 Inputs
VAR_INPUT
    Execute:                BOOL;
    RatioNumerator:         INT;
    RatioDenominator:       INT;
    Acceleration:           LREAL;
    Deceleration:           LREAL;
    Jerk:                   LREAL;  //from V3.0.5
    BufferMode:             MC_BufferMode_BkPlcMc:=Aborting_BkPlcMc;    //from V3.0.8
END_VAR

Name
Execute
RatioNumerator

Type
BOOL
INT

RatioDenominator

INT

Acceleration

LREAL

Deceleration

LREAL

Jerk
BufferMode

LREAL
MC_BufferMode_BkPlcMc

Description
A rising edge at this input starts the coupling.
[1, 1] These parameters describe the coupling
factor in the form of a gear unit.
[1, 1] These parameters describe the coupling
factor in the form of a gear unit.
[mm/s2] The acceleration permitted for the
synchronization in actual value units of the axis
per square second.
[mm/s2] The deceleration permitted for the
synchronization in actual value units of the axis
per square second.
[mm/s3] The jerk to be applied.
Reserved. This input is provided in preparation
for a future build. It should currently either not be
assigned or assigned the constant
Aborting_BkPlcMc. (from V3.0.8)

TF5810

Version: 1.8.3

63

MC_GearIn_BkPlcMcExecute  BOOLRatioNumerator  INTRatioDenominator  INTAcceleration  LREALDeceleration  LREALJerk  LREALBufferMode  MC_BufferMode_BkPlcMc↔Master  Reference To AXIS_REF_BkPlcMc↔Slave  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  InGearBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIdBOOL  ActivePLCopen Motion Control

 Inputs/outputs

VAR_IN_OUT
    Master:         AXIS_REF_BkPlcMc;
    Slave:          AXIS_REF_BkPlcMc;
END_VAR

Name
Master

Type
AXIS_REF_BkPlcMc

Slave

AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    InGear:         BOOL;
    CommandAborted: BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
    Active:         BOOL;
END_VAR

Name
Busy
InGear

CommandAborted
Error
ErrorID
Active

Type
BOOL
BOOL

BOOL
BOOL
UDINT
BOOL

Description
Indicates that a command is being processed.
This indicates the first successful synchronization of the axes. The signal
the remains active, even if the synchronization subsequently fails
temporarily or permanently.
This indicates abortion of the coupling.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.
Indicates that a command is being processed.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• Next, the system checks whether RatioDenominator is 0. In this case the system responds with Error

and ErrorID:=dwTcHydErrCdIllegalGearFactor.

• Currently, the coupling can only be activated if both the master and the slave are at standstill.

Otherwise the system responds with Error and ErrorID:=dwTcHydErrCdNotStartable.

• If the axis is already in an error state, or if it is in the process of carrying out a stop operation, it

responds with Error and ErrorID:=dwTcHydErrCdNotReady.

• If the motion algorithm is already indicating an error code, the system responds with Error and

ErrorID:= the motion algorithm's error code.

If these checks could be performed without problem, the coupling is initiated. The axis is now in state
McState_Synchronizedmotion [} 103], and the function block starts monitoring the coupling.

When the velocity required by the coupling is reached for the first time while the slave axis coupling is active,
this is indicated at output InGear. Since the coupling can currently only be activated at standstill, this is the
case immediately. If the slave axis is unable to follow the specifications for some reason while the coupling is
active, InGear remains unchanged.

If an error code occurs in the motion generator while the coupling is active, the system responds with Error
and ErrorID:=motion algorithm error code.

A falling edge at Execute clears all the pending output signals. If Execute is set to FALSE while the coupling
is still active, the existing coupling remains unaffected and active.

64

Version: 1.8.3

TF5810

PLCopen Motion Control

The output Active is currently identical to the output Busy.

4.2.8

MC_GearInPos_BkPlcMc

Available from version 3.0.33

The function block starts and monitors an on-the-fly coupling between two axes. To release the coupling, an
MC_GearOut_BkPlcMc [} 67] function block should be used.

 Inputs
VAR_INPUT
     Execute:               BOOL;
     RatioNumerator:        INT;
     RatioDenominator:      INT;
     MasterSyncPosition:    LREAL;
     SlaveSyncPosition:     LREAL;
     SyncMode:              INT;
     MasterStartDistance:   LREAL;
     Acceleration:          LREAL;
     Deceleration:          LREAL;
     Jerk:                  LREAL;   //from V3.0.5
     BufferMode:            MC_BufferMode_BkPlcMc:=Aborting_BkPlcMc;
END_VAR

TF5810

Version: 1.8.3

65

MC_GearInPos_BkPlcMcExecute  BOOLRatioNumerator  INTRatioDenominator  INTMasterSyncPosition  LREALSlaveSyncPosition  LREALSyncMode  INTMasterStartDistance  LREALAcceleration  LREALDeceleration  LREALJerk  LREALBufferMode  MC_BufferMode_BkPlcMc↔Master  Reference To AXIS_REF_BkPlcMc↔Slave  Reference To AXIS_REF_BkPlcMcBOOL  StartSyncBOOL  InSyncBOOL  BusyBOOL  ActiveBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIdPLCopen Motion Control

Name
Execute
RatioNumerator

Type
BOOL
INT

RatioDenominator

INT

MasterSyncPosition
SlaveSyncPosition
SyncMode
MasterStartDistance

Acceleration

Deceleration

Jerk
BufferMode

LREAL
LREAL
INT
LREAL

LREAL

LREAL

LREAL
MC_BufferMode_Bk
PlcMc

 Inputs/outputs

VAR_IN_OUT
     Master:       AXIS_REF_BkPlcMc;
     Slave:        AXIS_REF_BkPlcMc;
END_VAR

Description
A rising edge at this input starts the coupling.
[1, 1] These parameters describe the coupling factor in
the form of a gear unit.
[1, 1] These parameters describe the coupling factor in
the form of a gear unit.
[mm] The coupling is fully active from this master position.
[mm] The coupling is fully active from this slave position.
Currently not supported.
[mm] This is the master distance over which the coupling
is established.
[mm/s2] The acceleration permitted for the synchronization
in actual value units of the axis per square second.
[mm/s2] The deceleration permitted for the
synchronization in actual value units of the axis per
square second.
[mm/s3] The jerk to be applied.
Reserved. This input is provided in preparation for a
future build. It should currently either not be assigned or
assigned the constant Aborting_BkPlcMc. (from V3.0.8)

Name
Master

Type
AXIS_REF_BkPlcMc

Slave

AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
     StartSync:        BOOL;
     InSync:           BOOL;
     Busy:             BOOL;
     Active:           BOOL;
     CommandAborted:   BOOL;
     Error:            BOOL;
     ErrorID:          UDINT;
END_VAR

Name
StartSync

InSync

Busy
Active
CommandAborted
Error
ErrorID

Type
BOOL

BOOL

BOOL
BOOL
BOOL
BOOL
UDINT

Description
Indicates the transition phase between idle state and fully active
coupling.
This indicates the first successful synchronization of the axes. The signal
the remains active, even if the synchronization subsequently fails
temporarily or permanently.
Indicates that a command is being processed.
Indicates that a command is being processed.
This indicates abortion of the coupling.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

66

Version: 1.8.3

TF5810

PLCopen Motion Control

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• Next, the system checks whether RatioDenominator is 0. In this case the system responds with Error

and ErrorID:=dwTcHydErrCdIllegalGearFactor.

• If RatioDenominator is less than 0, the system responds with Error and

ErrorID:=dwTcHydErrCdNotSupport.

• The coupling can only be activated if the slave is at standstill. Otherwise the system responds with

Error and ErrorID:=dwTcHydErrCdNotStartable.

• If the absolute value of the MasterStartDistance is too small, the system responds with Error and

ErrorID:=dwTcHydErrCdCannotSynchronize.

• If the actual position of the master is not between MasterSyncPosition and the end of the

synchronization distance specified by MasterStartDistance, the system responds with Error and
ErrorID:=dwTcHydErrCdCannotSynchronize.

If these checks could be performed without problem, the coupling is initiated. The slave axis initially
continues to be in state McState_Standstill [} 103]. Only when the master axis reaches the start of the
synchronization distance for the first time does the slave axis report McState_Synchronizedmotion [} 103]
and indicate StartSync, and the function block starts monitoring the coupling. As soon as the axis reaches
the end the synchronization distance for the first time, the slave axis indicates InSync. Should the master
axis later pass the start of the synchronization distance backwards, the coupling is not released.

If an error code occurs in the motion generator while the coupling is active, the system responds with Error
and ErrorID:=motion algorithm error code.

A falling edge at Execute clears all the pending output signals. If Execute is set to FALSE while the coupling
is still active, the existing coupling remains unaffected and active.

An example is available under #103 [} 374].

The function block does not support the functionality of TwinCAT NC.

The output Active is currently identical to the output Busy.

4.2.9

MC_GearOut_BkPlcMc

Available from version 3.0

The function block releases a coupling between two axes. This coupling must have been established with an
MC_GearIn_BkPlcMc [} 63] or an MC_GearInPos_BkPlcMc [} 65] function block.

 Inputs
VAR_INPUT
    Execute:        BOOL;
END_VAR

TF5810

Version: 1.8.3

67

MC_GearOut_BkPlcMcExecute  BOOL↔Slave  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  ErrorUDINT  ErrorIdPLCopen Motion Control

Name
Execute

Type
BOOL

Description
A rising edge at this input releases the coupling.

 Inputs/outputs

VAR_IN_OUT
    Slave:          AXIS_REF_BkPlcMc;
END_VAR

Name
Slave

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    Done:           BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
Busy
Done
Error
ErrorID

Type
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
Successful processing of the movement is indicated here.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If the axis is not operated in a gear coupling, the function block immediately indicates Done and omits

all further checks or activities.

• If the current set velocity of the axis is smaller than the velocity specified by

pStAxParams.fCreepSpeed, the axis immediately assumes McState_Standstill and dissipates the
residual velocity. Done is indicated, and no further checks or activities take place.

If these checks could be performed without problem and Done is not already indicated for one of the reasons
mentioned, the motion controlled by the gear coupling is converted to a continuous motion with the same
velocity and direction, which is independent of the master. Done is indicated if this conversion was executed
successfully, otherwise the system responds with Error and ErrorID:=error code.

4.2.10

MC_Home_BkPlcMc

Available from version 3.0

This function block starts and monitors the homing of an axis.

68

Version: 1.8.3

TF5810

MC_Home_BkPlcMcExecute  BOOLPosition  LREALHomingMode  MC_HomingMode_BkPlcMcCalibrationCam  BOOLBufferMode  MC_BufferMode_BkPlcMcOldDebugTag  UDINT↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDBOOL  ActivePLCopen Motion Control

 Inputs
VAR_INPUT
    Execute:        BOOL;
    Position:       LREAL;
    HomingMode:     MC_HomingMode_BkPlcMc;
    CalibrationCam: BOOL;
    BufferMode:     MC_BufferMode_BkPlcMc:=Aborting_BkPlcMc;    //from V3.0.8
END_VAR

Name
Execute

Type
BOOL

Position
HomingMode

LREAL
MC_HomingMode_BkPlcMc

CalibrationCam

BOOL

BufferMode

MC_BufferMode_BkPlcMc

Description
The homing is initiated by a rising edge at this
input.
[mm] The reference position.

Specifies the method [} 125] to be used.
This can be used for direct transfer of the
referencing index (cam).
Reserved. This input is provided in preparation for
a future build. It should currently either not be
assigned or assigned the constant
Aborting_BkPlcMc. (from V3.0.8)

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    Done:           BOOL;
    CommandAborted: BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
Busy
Done
CommandAborted
Error
ErrorID

Type
BOOL
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
Successful processing of the homing is indicated here.
Abortion of homing is indicated here.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Execute, the function block examines the transferred axis interface. A number of
problems can be detected and reported during this process:

• Homing can only be started from a stationary condition without errors. If that is not the case, the

function block will react by asserting Error with ErrorID:=dwTcHydErrCdNotStartable or with the error
code that is passed to it.

• If the axis is already in a fault state, or if it is in the process of carrying out a stop operation, it responds

with Error and ErrorID:=dwTcHydErrCdNotReady.

• If one of the velocities stated in the axis parameters is too small (less than 1% of the reference velocity)

the function block responds with Error and ErrorID:=dwTcHydErrCdSetVelo.

TF5810

Version: 1.8.3

69

PLCopen Motion Control

Homing begins if these checks are carried out without problems. The exact sequence is specified by
HomingMode [} 125]. If the movement algorithm reports an error code while homing is being executed, the
function block responds with Error and ErrorID:=the movement algorithm's error code. If completion of
homing is prevented by the activity of another function block, the function block responds with
CommandAborted. Successful completion of homing is reported with Done.

A falling edge at Execute clears all the pending output signals. If, while homing is still active, Execute is set
to FALSE, execution of homing that had started continues unaffected. The signals provided at the end of the
movement (Error, ErrorID, CommandAborted, Done) are made available for one cycle.

Notice fEnc_DefaultHomePosition in pStAxParams is provided for circumstances in which the
application does not itself specify a reference position and a value saved with the machine data is to
be loaded for use instead. If different values are required, depending on the situation, use should be
made of fCustomerData[] in pStAxParams.

If iTcMc_EncoderSim is set as encoder type, the mode MC_Direct_BkPlcMc takes effect, irrespective of
HomingMode and AXIS_REF_BkPlcMc [} 86].stAxParams.nEnc_HomingType.

MC_DefaultHomingMode_BkPlcMc

The referencing method is not specified by the application, but through AXIS_REF_BkPlcMc
[} 86].stAxParams.nEnc_HomingType. The following rules apply:

nEnc_HomingType
iTcMc_HomingOnBlock
iTcMc_HomingOnIndex
iTcMc_HomingOnSync
iTcMc_HomingOnExec

MC_AbsSwitch_BkPlcMc

MC_HomingMode_BkPlcMc
MC_Block_BkPlcMc
MC_AbsSwitch_BkPlcMc
MC_RefPulse_BkPlcMc
MC_Direct_BkPlcMc

The axis is moved with AXIS_REF_BkPlcMc [} 86].stAxParams.fEnc_RefIndexVelo in the direction specified
by bEnc_RefIndexPositive. The axis stops if CalibrationCam becomes TRUE or if the reference cam (bit 5,
dwTcHydDcDwRefIndex) is detected in AXIS_REF_BkPlcMc [} 86].stAxRtData.nDeCtrlDWord. The axis is
then moved with fEnc_RefSyncVelo in the direction specified by bEnc_RefSyncPositive, until the reference
cam is exited. The actual value for the axis is set to the value of the reference position.

MC_LimitSwitch_BkPlcMc

Not currently supported.

MC_RefPulse_BkPlcMc

The axis is moved with AXIS_REF_BkPlcMc [} 86].stAxParams.fEnc_RefIndexVelo in the direction specified
by bEnc_RefIndexPositive. The axis stops if CalibrationCam becomes TRUE or if the reference cam (bit 5,
dwTcHydDcDwRefIndex) is detected in AXIS_REF_BkPlcMc [} 86].stAxRtData.nDeCtrlDWord. The axis is
then moved with fEnc_RefSyncVelo in the direction specified by bEnc_RefSyncPositive, until the reference
cam is exited. The encoder's hardware latch is then activated, and the axis is moved on until the latch
becomes valid. After the axis has stopped, the actual value for the axis is set to a value that is calculated
from the reference position and from the distance covered since the encoder's sync pulse was detected.

MC_Direct_BkPlcMc

The actual value of the axis is immediately set to the value of the reference position.

MC_Absolute_BkPlcMc

Not currently supported.

70

Version: 1.8.3

TF5810

PLCopen Motion Control

MC_Block_BkPlcMc

The axis is moved with AXIS_REF_BkPlcMc [} 86].stAxParams.fEnc_RefIndexVelo in the direction specified
by bEnc_RefIndexPositive. If no movement is detected over a period of 2 seconds, the fixed stop (block) is
considered to have been reached. The actual value for the axis is set to the value of the reference position.

From version 3.0.41 of 12 October 2017 it is possible to change the time period for the function block
detection. See ST_TcHydAxRtData [} 141].fBlockDetectDelay.

MC_FlyingSwitch_BkPlcMc

Not currently supported.

MC_FlyingRefPulse_BkPlcMc

Not currently supported.

4.2.11

MC_Halt_BkPlcMc

Available from version 3.0

The function block cancels a current axis motion and monitors the stop operation.

The stop operation initiated by this function block can be interrupted by other function blocks. An
MC_Stop_BkPlcMc function block can be used to prevent the axis from restarting during a stop
operation.

 Inputs
VAR_INPUT
    Execute:        BOOL;
    Deceleration:   LREAL;  //from V3.0.5
    Jerk:           LREAL;  //from V3.0.5
    RampTime:       LREAL;  //from V3.0.5
    BufferMode:     MC_BufferMode_BkPlcMc:=Aborting_BkPlcMc;    //from V3.0.8
END_VAR

Name
Execute

Type
BOOL

Deceleration
Jerk
RampTime
BufferMode

LREAL
LREAL
LREAL
MC_BufferMode_BkPlcMc

Description
A rising edge at this input ends a movement being
carried out by the axis.
[mm/s2] The deceleration to be applied.
[mm/s3] The jerk to be applied.
[s] The required stopping time.
Reserved. This input is provided in preparation for a
future build. It should currently either not be assigned or
assigned the constant Aborting_BkPlcMc. (from V3.0.8)

TF5810

Version: 1.8.3

71

MC_Halt_BkPlcMcExecute  BOOLDeceleration  LREALJerk  LREALRampTime  LREALBufferMode  MC_BufferMode_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIdBOOL  ActivePLCopen Motion Control

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    Done:           BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
    Active:         BOOL;
    CommandAborted: BOOL;
END_VAR

Name
Busy
Done
Error
ErrorID
Active
CommandAborted

Type
BOOL
BOOL
BOOL
UDINT
BOOL
BOOL

Description
Indicates that a command is being processed.
This indicates successful processing of the operation.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.
Indicates that a command is being processed.
Indicates that processing of this command was aborted by another
command.

Behavior of the function block

The behavior of the function block is identical to that of the MC_Stop_BkPlcMc [} 82]() function block. The
only difference is that processing of the command can be aborted by other function blocks.

4.2.12

MC_ImediateStop_BkPlcMc

Available from version 3.0.5

The function block cancels a current axis motion.

 Inputs
VAR_INPUT
    Execute:        BOOL;
END_VAR

Name
Execute

Type
BOOL

Description
A rising edge at this input ends a movement being carried out by the axis.

72

Version: 1.8.3

TF5810

MC_ImediateStop_BkPlcMcExecute  BOOL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  ErrorUDINT  ErrorIdBOOL  ActiveBOOL  CommandAbortedPLCopen Motion Control

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    Done:           BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
    Active:         BOOL;
    CommandAborted: BOOL;
END_VAR

Name
Busy
Done
Error
ErrorID
Active
CommandAborted

Type
BOOL
BOOL
BOOL
UDINT
BOOL
BOOL

Description
Indicates that a command is being processed.
This indicates successful processing of the operation.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.
Indicates that a command is being processed.
Indicates that processing of this command was aborted by another
command.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• The stop can only be executed if the axis is actively carrying out a movement. If it is stationary, the

function block immediately asserts the Done signal.

• If the axis is already in an error state, or if it is in the process of carrying out a stop operation, the

system responds with Error and ErrorID:=dwTcHydErrCdNotReady.

• If the axis is in a state, in which it is controlled by a coupling with another axis or a comparable

mechanism, it responds with Error and ErrorID:=dwTcHydErrCdNotReady.

The Stop operation begins if these checks can be carried out without problems. The control value of the axis
is immediately set to 0, without any ramp. All outputs of control or regulation voltages are then suppressed,
as long as Execute is set to TRUE.

4.2.13

MC_MoveAbsolute_BkPlcMc

TF5810

Version: 1.8.3

73

MC_MoveAbsolute_BkPlcMcExecute  BOOLPosition  LREALVelocity  LREALAcceleration  LREALDeceleration  LREALJerk  LREALDirection  MC_Direction_BkPlcMcBufferMode  MC_BufferMode_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDBOOL  ActivePLCopen Motion Control

Available from version 3.0

This function block starts and monitors the movement of an axis.

 Inputs
VAR_INPUT
    Execute:        BOOL;
    Position:       LREAL;
    Velocity:       LREAL;
    Acceleration:   LREAL;
    Deceleration:   LREAL;
    Jerk:           LREAL;
    Direction:      MC_Direction_BkPlcMc:=MC_Shortest_Way_BkPlcMc;   //from V3.0.8
    BufferMode:     MC_BufferMode_BkPlcMc:=Aborting_BkPlcMc;        //from V3.0.8
END_VAR

Name
Execute
Position

Type
BOOL
LREAL

Velocity

LREAL

Acceleration

LREAL

Deceleration

LREAL

Jerk
Direction

LREAL
MC_Direction_BkPlcMc

BufferMode

MC_BufferMode_BkPlcMc

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Description
The movement is initiated by a rising edge at this input.
[mm] The target position of the movement in actual
value units of the axis.
[mm/s] The required motion velocity in actual value
units of the axis per second.
[mm/s2] The required acceleration in actual value units
of the axis per square second. If this parameter is 0.0, it
is replaced by a default value from the axis parameters.
[mm/s2] The required deceleration in actual value units
of the axis per square second. If this parameter is 0.0, it
is replaced by a default value from the axis parameters.
Reserved [mm/s3]
Reserved. This input was only amended for
compatibility reasons and either should not be
assigned, or the constant MC_Shortest_Way_BkPlcMc
should be assigned to it. (from V3.0.8)
Reserved. This input is provided in preparation for a
future build. It should currently either not be assigned or
assigned the constant Aborting_BkPlcMc. (from V3.0.8)

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    Done:           BOOL;
    CommandAborted: BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

74

Version: 1.8.3

TF5810

PLCopen Motion Control

Name
Busy
Done
CommandAborted
Error
ErrorID

Type
BOOL
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
Successful processing of the movement is indicated here.
Abortion of the movement is indicated here.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• The possibility that Position is located behind an active software limit switch is checked next. In this

case the system responds with Error and ErrorID:=dwTcHydErrCdSoftEnd.

• Depending on the motion algorithm specified in Axis.pStAxParams^.nProfile the axis may either only
be able to begin the movement initiated here when stationary, or may be able to begin from another
movement that has not yet been completed. If it is unable at the present time to accept this new order,
the system responds with Error and ErrorID:=dwTcHydErrCdNotStartable.

• If the axis is already in an error state, or if it is in the process of carrying out a stop operation, it

responds with Error and ErrorID:=dwTcHydErrCdNotReady.

• If Velocity is too small (less than 1% of the reference velocity) the system responds with Error and

ErrorID:=dwTcHydErrCdSetVelo.

• If Acceleration is too small (the Velocity cannot be reached within 100 seconds) the system responds

with Error and ErrorID:=dwTcHydErrCdAcc.

• If Deceleration is too small (the Velocity cannot be reduced within 100 seconds) the system responds

with Error and ErrorID:=dwTcHydErrCdAcc.

• If the motion algorithm is already indicating an error code, the system responds with Error and

ErrorID:= the motion algorithm's error code.

The movement begins if these checks can be carried out without problems. This is done by limiting the
parameters Position, Velocity, Acceleration and Deceleration to the maximum permissible values and
passing them to the motion algorithm. The axis is now in the McState_DiscreteMotion [} 103] state, and the
function block begins to monitor the movement.

If the motion algorithm reports an error code while the movement is being executed, the system responds
with Error and ErrorID:=the motion algorithm's error code. If completion of the movement is prevented by
the activity of another function block, the system responds with CommandAborted. If the motion algorithm
achieves the target conditions for the axis, the system responds with Done.

A falling edge at Execute clears all the pending output signals. If, while the movement is still active, Execute
is set to FALSE, execution of the movement that had started continues unaffected. The signals provided at
the end of the movement (Error, ErrorID, CommandAborted, Done) are made available for one cycle.

4.2.14

MC_MoveJoySticked_BkPlcMc

Available from version 3.0

This function block starts and monitors the movement of an axis.

TF5810

Version: 1.8.3

75

MC_MoveJoySticked_BkPlcMcExecute  BOOLJoyStick  LREALAcceleration  LREALDeceleration  LREALJerk  LREAL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIdBOOL  ActivePLCopen Motion Control

This function is currently only supported by axes, which are controlled by a function block of type
MC_AxRuntimeCtrlBased_BkPlcMc (in preparation: MC_AxRunTimeTimeRamp_BkPlcMc). Such a
function block is selected by specifying the corresponding constant from E_TcMcProfileType under
nProfileType in ST_TcHydAxParam.

 Inputs
VAR_INPUT
    Execute:        BOOL;
    JoyStick:       LREAL;
    Acceleration:   LREAL;
    Deceleration:   LREAL;
    Jerk:           LREAL;
END_VAR

Name
Execute
JoyStick
Acceleration

Type
BOOL
LREAL
LREAL

Deceleration

LREAL

Jerk

LREAL

Description
The movement is initiated by a rising edge at this input.
[1] The velocity specified via the control unit, normalized to the range ±1.0.
[mm/s2] The required acceleration in actual value units of the axis per square
second.
[mm/s2] The required deceleration in actual value units of the axis per square
second.
Reserved [mm/s3]

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    CommandAborted: BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
Busy
CommandAborted
Error
ErrorID

Type
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
Abortion of the movement is indicated here.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If the axis is already in an error state, or if it is in the process of carrying out a stop operation, it

responds with Error and ErrorID:=dwTcHydErrCdNotReady.

• If the motion algorithm is already indicating an error code, the system responds with Error and

ErrorID:= the motion algorithm's error code.

• Next, the system checks whether the generator of the axis supports the required function. If this is not

the case, the system responds with is Error and ErrorID:=dwTcHydErrCdNotCompatible.

76

Version: 1.8.3

TF5810

PLCopen Motion Control

The movement begins if these checks can be carried out without problems. To this end the motion algorithm
is set to state iTcHydStateExtGenerated and the axis to state McState_Synchronizedmotion. The axis
velocity is specified through JoyStick and ST_TcHydAxParam [} 130].fRefVelo. Changes in velocity are
accompanied by ramp limitation to ST_TcHydAxParam [} 130].fMaxAcc. If the axis moves towards an active
software limit switch, the velocity is limited, depending on the remaining distance, such that the limit switch is
approached correctly.

A falling edge at Execute offset puts motion algorithm in state iTcHydStateTcDecP or iTcHydStateTcDecM
and the axis in state McState_Standstill. If the axis is in motion at this point in time, it is decelerated with a
stop ramp and assumes state iTcHydStateIdle.

4.2.15

MC_MoveRelative_BkPlcMc

Available from version 3.0

This function block starts and monitors the movement of an axis.

 Inputs
VAR_INPUT
    Execute:        BOOL;
    Distance:       LREAL;
    Velocity:       LREAL;
    Acceleration:   LREAL;
    Deceleration:   LREAL;
    Jerk:           LREAL;
    BufferMode:     MC_BufferMode_BkPlcMc:=Aborting_BkPlcMc;    //from V3.0.8
END_VAR

Name
Execute
Distance

Type
BOOL
LREAL

Velocity

LREAL

Acceleration

LREAL

Deceleration

LREAL

Jerk
BufferMode

LREAL
MC_BufferMode_BkPlcMc

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Description
The movement is initiated by a rising edge at this input.
[mm] The distance to the target position of the
movement in actual value units of the axis.
[mm/s] The required motion velocity in actual value
units of the axis per second.
[mm/s2] The required acceleration in actual value units
of the axis per square second.
[mm/s2] The required deceleration in actual value units
of the axis per square second.
Reserved [mm/s3]
Reserved. This input is provided in preparation for a
future build. It should currently either not be assigned or
assigned the constant Aborting_BkPlcMc. (from V3.0.8)

TF5810

Version: 1.8.3

77

MC_MoveRelative_BkPlcMcExecute  BOOLDistance  LREALVelocity  LREALAcceleration  LREALDeceleration  LREALJerk  LREALBufferMode  MC_BufferMode_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDBOOL  ActivePLCopen Motion Control

Name
Axis

Type
AXIS_REF_BkPlcMc

Description
Here, the address of a variable of type
AXIS_REF_BkPlcMc [} 86] should be transferred.

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    Done:           BOOL;
    CommandAborted: BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
Busy
Done
CommandAborted
Error
ErrorID

Type
BOOL
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
Successful processing of the movement is indicated here.
Abortion of the movement is indicated here.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• The possibility that moving by Distance will lead to a conflict with an active software limit switch is
checked next. In this case the system responds with Error and ErrorID:=dwTcHydErrCdSoftEnd.

• Depending on the motion algorithm specified in Axis.pStAxParams^.nProfile the axis may either only
be able to begin the movement initiated here when stationary, or may be able to begin from another
movement that has not yet been completed. If it is unable at the present time to accept this new order,
the system responds with Error and ErrorID:=dwTcHydErrCdNotStartable.

• If the axis is already in an error state, or if it is in the process of carrying out a stop operation, it

responds with Error and ErrorID:=dwTcHydErrCdNotReady.

• If Velocity is too small (less than 1% of the reference velocity) the system responds with Error and

ErrorID:=dwTcHydErrCdSetVelo.

• If Acceleration is too small (the Velocity cannot be reached within 100 seconds) the system responds

with Error and ErrorID:=dwTcHydErrCdAcc.

• If Deceleration is too small (the Velocity cannot be reduced within 100 seconds) the system responds

with Error and ErrorID:=dwTcHydErrCdAcc.

• If the motion algorithm is already indicating an error code, the system responds with Error and

ErrorID:= the motion algorithm's error code.

The movement begins if these checks can be carried out without problems. This is done by limiting the
parameters Distance, Velocity, Acceleration and Deceleration to the maximum permissible values and
passing them to the motion algorithm. The axis is now in the McState_DiscreteMotion [} 103] state, and the
function block begins to monitor the movement.

If the motion algorithm reports an error code while the movement is being executed, the system responds
with Error and ErrorID:=the motion algorithm's error code. If completion of the movement is prevented by
the activity of another function block, the system responds with CommandAborted. If the motion algorithm
achieves the target conditions for the axis, the system responds with Done.

A falling edge at Execute clears all the pending output signals. If, while the movement is still active, Execute
is set to FALSE, execution of the movement that had started continues unaffected. The signals provided at
the end of the movement (Error, ErrorID, CommandAborted, Done) are made available for one cycle.

78

Version: 1.8.3

TF5810

4.2.16

MC_MoveVelocity_BkPlcMc

PLCopen Motion Control

Available from version 3.0

This function block starts and monitors the movement of an axis.

 Inputs
VAR_INPUT
    Execute:        BOOL;
    Velocity:       LREAL;
    Acceleration:   LREAL;
    Deceleration:   LREAL;
    Direction:      MC_Direction_BkPlcMc;
    BufferMode:     MC_BufferMode_BkPlcMc:=Aborting_BkPlcMc;    //from V3.0.8
END_VAR

Name
Execute
Velocity

Type
BOOL
LREAL

Acceleration

LREAL

Deceleration

LREAL

Direction

MC_Direction_BkPlcMc

BufferMode

MC_Direction_BkPlcMc

Description
The movement is initiated by a rising edge at this input.
[mm/s] The required motion velocity in actual value units of
the axis per second.
[mm/s2] The required acceleration in actual value units of
the axis per square second.
[mm/s2] The required deceleration in actual value units of
the axis per square second.
A direction specification coded according to
MC_Direction_BkPlcMc [} 124].
Reserved. This input is provided in preparation for a future
build. It should currently either not be assigned or
assigned the constant Aborting_BkPlcMc. (from V3.0.8)

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    InVelocity:     BOOL;
    CommandAborted: BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc
[} 86] should be transferred.

TF5810

Version: 1.8.3

79

MC_MoveVelocity_BkPlcMcExecute  BOOLVelocity  LREALAcceleration  LREALDeceleration  LREALJerk  LREALDirection  MC_Direction_BkPlcMcBufferMode  MC_BufferMode_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  InVelocityBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIdBOOL  ActivePLCopen Motion Control

Name
Busy
InVelocity

CommandAborted
Error
ErrorID

Type
BOOL
BOOL

BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
This output becomes TRUE when the axis reaches the required velocity
for the first time.
Abortion of the movement is indicated here.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• Depending on the motion algorithm specified in Axis.pStAxParams^.nProfile the axis may either only
be able to begin the movement initiated here when stationary, or may be able to begin from another
movement that has not yet been completed. If it is unable at the present time to accept this new order,
the system responds with Error and ErrorID:=dwTcHydErrCdNotStartable.

• If the axis is already in an error state, or if it is in the process of carrying out a stop operation, it

responds with Error and ErrorID:=dwTcHydErrCdNotReady.

• If Velocity is too small (less than 1% of the reference velocity) the system responds with Error and

ErrorID:=dwTcHydErrCdSetVelo.

• If the motion algorithm is already indicating an error code, the system responds with Error and

ErrorID:= the motion algorithm's error code.

The movement begins if these checks can be carried out without problems. This is done by selecting a value
for the target position depending on Direction and the parameters for the software limit switches. This is
done by limiting the parameters Velocity, Acceleration and Deceleration to the maximum permissible
values and passing them to the motion algorithm. The axis is now in the McState_Continousmotion [} 103]
state, and the function block begins to monitor the movement.

If the motion algorithm reports an error code while the movement is being executed, the system responds
with Error and ErrorID:=the motion algorithm's error code. If completion of the movement is prevented by
the activity of another function block, the system responds with CommandAborted. InVelocity is set when
the motion algorithm reaches the required velocity.

A falling edge at Execute clears all the pending output signals. If, while the movement is still active, Execute
is set to FALSE, execution of the movement that had started continues unaffected. The signals provided at
the end of the movement (Error, ErrorID, CommandAborted, InVelocity) are made available for one cycle.

4.2.17

MC_RampedStop_BkPlcMc

The function block cancels a movement that is currently being executed.

 Inputs
VAR_INPUT
    Execute:        BOOL;
    RampTime:       LREAL;  //from V3.0.5
END_VAR

80

Version: 1.8.3

TF5810

MC_RampedStop_BkPlcMcExecute  BOOLRampTime  LREAL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  ErrorUDINT  ErrorIdBOOL  ActiveBOOL  CommandAbortedName
Execute
RampTime

Type
BOOL
LREAL

Description
A rising edge at this input ends a movement being carried out by the axis.
[s] The required stopping time.

PLCopen Motion Control

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    Done:           BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
    Active:         BOOL;
    CommandAborted: BOOL;
END_VAR

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc
[} 86] should be transferred.

Name
Busy
Done
Error
ErrorID
Active
CommandAborted

Type
BOOL
BOOL
BOOL
UDINT
BOOL
BOOL

Description
Indicates that a command is being processed.
This indicates successful processing of the operation.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.
Indicates that a command is being processed.
Indicates that processing of this command was aborted by another
command.

Behavior of the function block

On a rising edge at Execute, the function block examines the transferred axis interface. A number of
problems can be detected and reported during this process:

• The stop can only be executed if the axis is actively carrying out a movement. If it is stationary, the

function block immediately reports the Done signal.

• If the axis is already in an error state, or if it is in the process of carrying out a stop operation, it

responds with Error and ErrorID:=dwTcHydErrCdNotReady.

• If the axis is in a state, in which it is controlled by a coupling with another axis or a comparable

mechanism, it responds with Error and ErrorID:=dwTcHydErrCdNotReady.

The Stop operation begins if these checks can be carried out without problems. RampTime is used to
calculate a deceleration, taking into account the reference velocity. With this deceleration, the target velocity
is reduced to 0 with a pure time ramp.

No defined end position is driven to and the axis can overrun a software limit switch.

 CAUTION

TF5810

Version: 1.8.3

81

PLCopen Motion Control

4.2.18

MC_Stop_BkPlcMc

Available from version 3.0

The function block cancels a current axis motion and monitors the stop operation.

The stop operation initiated by this function block cannot be interrupted by other function blocks. A
function block MC_Halt_BkPlcMc should be used to enable an axis restart during a stop operation.

 Inputs
VAR_INPUT
    Execute:        BOOL;
    Deceleration:   LREAL;  //from V3.0.5
    Jerk:           LREAL;  //from V3.0.5
    RampTime:       LREAL;  //from V3.0.5
    BufferMode:     MC_BufferMode_BkPlcMc:=Aborting_BkPlcMc;    //from V3.0.8
END_VAR

Name
Execute

Type
BOOL

Deceleration
Jerk
RampTime
BufferMode

LREAL
LREAL
LREAL
MC_BufferMode_BkPlcMc

Description
A rising edge at this input ends a movement being
carried out by the axis.
[mm/s2] The deceleration to be applied.
[mm/s3] The jerk to be applied.
[s] The required stopping time.
Reserved. This input is provided in preparation for a
future build. It should currently either not be assigned or
assigned the constant Aborting_BkPlcMc. (from V3.0.8)

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    Done:           BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
    Active:         BOOL;
    CommandAborted: BOOL;
END_VAR

82

Version: 1.8.3

TF5810

MC_Stop_BkPlcMcExecute  BOOLDeceleration  LREALJerk  LREALRampTime  LREALBufferMode  MC_BufferMode_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  ErrorUDINT  ErrorIdBOOL  ActiveBOOL  CommandAbortedPLCopen Motion Control

Name
Busy
Done
Error
ErrorID
Active
CommandAborted

Type
BOOL
BOOL
BOOL
UDINT
BOOL
BOOL

Description
Indicates that a command is being processed.
This indicates successful processing of the operation.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.
Indicates that a command is being processed.
Indicates that processing of this command was aborted by another
command.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• The stop can only be executed if the axis is actively carrying out a movement. If it is stationary, the

function block immediately asserts the Done signal.

• If the axis is already in an error state, or if it is in the process of carrying out a stop operation, it

responds with Error and ErrorID:=dwTcHydErrCdNotReady.

• If the axis is in a state, in which it is controlled by a coupling with another axis or a comparable

mechanism, it responds with Error and ErrorID:=dwTcHydErrCdNotReady.

The Stop operation begins if these checks can be carried out without problems. Deceleration is used, if this
parameter is recognizably greater than 0. Otherwise RampTime is used to calculate a deceleration, taking
into account the reference velocity. If a jerk-limiting control value generator is selected, Jerk is used if this
parameter is recognizably greater than 0. If none of the mentioned parameters is recognizably greater than
0, the axis parameter MaxDec and MaxJerk are used.

The next reachable position is determined and used as new target position, taken into account the current
set velocity and the currently valid parameters. Once this position has been reached, the axis assumes its
regular behavior in idle state.

The RampTime specifies the time during which the axis is to be decelerated from its reference
speed to standstill. If the axis moves with a different velocity, the braking time reduces accordingly.
If control value generators with creep mode are used, the corresponding time is added to the
braking time.

If the motion algorithm reports an error code while the movement is being executed, the system responds
with Error and ErrorID:=the motion algorithm's error code. If the complete processing is prevented by the
activity of another function block, the system responds with CommandAborted. Successful completion of
the operation is reported with Done.

A falling edge at Execute clears all the pending output signals. If Execute is set to FALSE while the
operation is still active, the initiated stop continues unaffected. The signals provided at the end of the
movement (Error, ErrorID, Done) are made available for one cycle.

The output Active is currently identical to the output Busy.

TF5810

Version: 1.8.3

83

PLCopen Motion Control

4.2.19

MC_MoveJog_BkPlcMc

This function block starts and monitors the movement of an axis.

 Inputs
VAR_INPUT
    JogForward:      BOOL;
    JogBackwards:    BOOL;
    Mode:            E_TcMcJogMode;
    Position:        LREAL;
    Velocity:        LREAL;
    Acceleration:    LREAL;
    Deceleration:    LREAL;
    Jerk:            LREAL;
END_VAR

Name
JogForward

Type
BOOL

JogBackwards

BOOL

Mode

E_TcMcJogMode

Position

Velocity

LREAL

LREAL

Acceleration

LREAL

Deceleration

LREAL

Jerk

LREAL

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Description
The command is executed with rising edge and the axis moved
in positive direction of travel. Depending on the mode the axis
moves as long as the signal remains TRUE or stops
automatically after a specified distance. During the motion no
further signal edges are accepted (this includes the
JogBackwards input). If signal edges occur simultaneously at the
JogForward and JogBackwards inputs, JogForward has priority.
The command is executed with rising edge and the axis moved
in negative direction of travel. JogForward and JogBackwards
should be triggered alternatively, although they are also mutually
locked internally.
The input defines the E_TcMcJogMode operation mode in which
the manual function is to be executed.
[mm] Relative distance for movements in
MC_JOGMODE_INCHING operation mode.
[mm/s] The required motion velocity in actual value units of the
axis per second.
[mm/s2] The required acceleration in actual value units of the axis
per square second. If this parameter is 0.0, it is replaced by a
default value from the axis parameters.
[mm/s2] The required deceleration in actual value units of the
axis per square second. If this parameter is 0.0, it is replaced by
a default value from the axis parameters.
Reserved [mm/s3]

84

Version: 1.8.3

TF5810

MC_MoveJog_BkPlcMcJogForward  BOOLJogBackwards  BOOLMode  E_TcMcJogModePosition  LREALVelocity  LREALAcceleration  LREALDeceleration  LREALJerk  LREAL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  DoneBOOL  BusyBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDName
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

PLCopen Motion Control

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    Done:           BOOL;
    CommandAborted: BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
Busy
Done
CommandAborted
Error
ErrorID

Type
BOOL
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
Successful processing of the movement is indicated here.
Abortion of the movement is indicated here.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Direction: Reserved. This input was only amended for compatibility reasons and either should not be
assigned, or the constant MC_Shortest_Way_BkPlcMc should be assigned to it. (from V3.0.8)

BufferMode: Reserved. This input is provided in preparation for a future build. It should currently either not
be assigned or assigned the constant Aborting_BkPlcMc. (from V3.0.8)

Behavior of the function block

On a rising edge at JogForward or JogBackwards a movement is started, depending on the mode [} 165]
used.

• MC_JOGMODE_STANDARD_SLOW: triggers a MC_MoveVelocity_BkPlcMc [} 79] on a rising edge

and a MC_Stop_BkPlcMc [} 82] on a falling edge.

• MC_JOGMODE_STANDARD_FAST: triggers a MC_MoveVelocity_BkPlcMc on a rising edge and a

MC_Stop_BkPlcMc on a falling edge.

• MC_JOGMODE_CONTINOUS: triggers a MC_MoveVelocity_BkPlcMc on a rising edge and a

MC_Stop_BkPlcMc on a falling edge.

• MC_JOGMODE_INCHING: triggers a MC_MoveRelative_BkPlcMc [} 77] on a rising edge.

The lower-level function blocks check the higher-level axis interface and report the problems at the output
Error and ErrorID

The movement begins if these checks can be carried out without problems.

If the motion algorithm reports an error code while the movement is being executed, the system responds
with Error and ErrorID:=the motion algorithm's error code. If completion of the movement is prevented by
the activity of another function block, the system responds with CommandAborted. If the motion algorithm
achieves the target conditions for the axis, the system responds with Done.

A falling edge at Execute clears all the pending output signals. If, while the movement is still active, Execute
is set to FALSE, execution of the movement that had started continues unaffected. The signals provided at
the end of the movement (Error, ErrorID, CommandAborted, Done) are made available for one cycle.

TF5810

Version: 1.8.3

85

PLCopen Motion Control

4.3

Data types

4.3.1

AXIS_REF_BkPlcMc

Available from version 3.0

The variables in this structure consolidate the subcomponents of the axis. A variable of this type is
transferred to most function blocks of the library. This type therefore corresponds to the AXIS_REF data type
of PLCopen.

Syntax
TYPE AXIS_REF_BkPlcMc:
STRUCT
    sAxisName:          STRING(83) := 'NoName';
    pStAxLogBuffer:     POINTER TO ST_TcMcLogBuffer:=0;
    pStDeviceInput:     POINTER TO ST_TcPlcDeviceInput:=0;
    pStDeviceOutput:    POINTER TO ST_TcPlcDeviceOutput:=0;
    pStAxAuxLabels:     POINTER TO ST_TcMcAuxDataLabels:=0;
    pStAxAutoParams:    POINTER TO ST_TcMcAutoIdent:=0;
    pStAxCommandBuf:    POINTER TO ST_TcPlcCmdBuffer_BkPlcMc:=0;
    nActiveRequest:    UDINT := 0;
    nNextRequest:      UDINT := 1;
    bParamsEnable:     BOOL:=FALSE;
    nState:            E_TcMCFbState:=McState_Standstill;
    nInitState:        INT:=0;
    nInitError:        DINT:=0;
    nInterfaceType:    UINT := 16#FFFF;
    nDeviceInType:     UINT := 16#FFFF;
    nDeviceOutType:    UINT := 16#FFFF;
    nRtDataType:       UINT := 16#FFFF;
    nParamType:        UINT := 16#FFFF;
    nLogBufferType:    UINT := 16#FFFF;
    nAxAutoIdentType:  UINT := 16#FFFF;
    nCmdBufferType:    UINT := 16#FFFF;
    nLogLevel:         DINT := 0;
    nDebugTag:         UDINT := 16#00000000;
    stAxParams:        ST_TcHydAxParam;
    stAxRtData:        ST_TcHydAxRtData;
END_STRUCT
END_TYPE

86

Version: 1.8.3

TF5810

Parameter

PLCopen Motion Control

TF5810

Version: 1.8.3

87

PLCopen Motion Control

Name
sAxisName
pStAxLogBuffer

Type
STRING
POINTER TO
ST_TcMcLogBuffer

pStDeviceInput

POINTER TO
ST_TcPlcDeviceInput

pStDeviceOutput

POINTER TO
ST_TcPlcDeviceOutput

pStAxAuxLabels

POINTER TO
ST_TcMcAuxDataLabels

pStAxAutoParams

POINTER TO
ST_TcMcAutoIdent

pStAxCommandBuf

POINTER TO
ST_TcPlcCmdBuffer_Bk
PlcMc

nActiveRequest

UDINT

nNextRequest

UDINT

bParamsEnable

BOOL

nState

E_TcMCFbState

nInitState
nInitError
nInterfaceType

nDeviceInType

INT
DINT
UINT

UINT

nDeviceOutType

UINT

nRtDataType

UINT

Description
The text name of the axes.

The address of a variable of type ST_TcMcLogBuffer
[} 156]. This variable contains the LogBuffer of the
library.

The address of a variable of type ST_TcPlcDeviceInput
[} 149]. This variable contains all input interfaces of the
axis.

The address of a variable of type ST_TcPlcDeviceOutput
[} 153]. This variable contains all output interfaces of
the axis.

The address a variable of type ST_TcMcAuxDataLabels
[} 149]. This variable optionally contains the application
parameter IDs in
ST_TcHydAxParam:fCustomerData[..].

The address a variable of type ST_TcMcAutoIdent
[} 128]. This variable optionally contains the parameters
for an MC_AxUtiAutoIdent_BkPlcMc function block.
From V3.0.8 the input BufferMode defined by the
PLCopen is available for various function blocks. The
functionality that can be controlled with this is currently
in preparation. In this context this command buffer was
amended.
Every function block sets a code here that starts a
function on this axis. After this, the function block
monitors this variable to see if it is changed by another
function block that is taking over control through
another function. In this way any function block can tell
whether a function that it has started has been
interrupted by another function block, and can generate
appropriate signals.
Reserved. Used for generating new values for
nActiveRequest.
This variable is only TRUE if the parameters have been
placed into a valid state by being loaded from the file.
Saving the parameters will also assert this signal,
because this also ensures consistency between the
data in the parameter structure and in the file. The axis
is not ready to operate while this variable is not TRUE.

At runtime, write accesses to the parameter
structure temporarily set this variable to FALSE,
after which it is returned to its previous state.
The current state of the axis is stored here, encoded in
accordance with E_TcMCFbState [} 103].
The current state of the initialization.
Any error code detected during initialization.
The type code of the currently valid
AXIS_REF_BkPlcMc variable type.
The type code of the currently valid
ST_TcPlcDeviceInput [} 149] variable type.
The type code of the currently valid
ST_TcPlcDeviceOutput [} 153] variable type.

The type code of the currently valid ST_TcHydAxRtData
[} 141] variable type.

88

Version: 1.8.3

TF5810

Name
nParamType

Type
UINT

nLogBufferType

UINT

nAxAutoIdentType

UINT

nCmdBufferType

nLogLevel

UINT

DINT

nDebugTag

UDINT

stAxParams

ST_TcHydAxParam

stAxRtData

ST_TcHydAxRtData

PLCopen Motion Control

Description

The type code of the currently valid ST_TcHydAxParam
[} 130] variable type.

The type code of the currently valid ST_TcMcLogBuffer
[} 156] variable type.

The type code of the currently valid ST_TcMcAutoIdent
[} 128] variable type.
reserved. The type code of the currently valid command
buffer variable type.

The Message Level [} 347], from which entries in the
logging buffer are to be made.
Many library blocks enter a debug ID here for the
duration of their execution.

This variable of type ST_TcHydAxParam [} 130] contains
the axis parameters.

This variable of type ST_TcHydAxRtData [} 141]
contains the runtime data of the axis.

In order to make the data structures of the library independent of the CPU architecture (I86, Strong
ARM), it is necessary to change the order of data or insert placeholders in some places. These
placeholders contain a name in the form "bAlign_1"; the number has no purpose. Neither existence,
name, type or dimensioning are guaranteed.

4.3.2

E_TcPlcBufferedCmdType_BkPlcMc

The constants in this list are used to identify buffered axis commands. See MC_BufferMode_BkPlcMc [} 121].

Syntax
TYPE E_TcPlcBufferedCmdType_BkPlcMc : (
(* last modification: xx.xx.2009 *)
iBufferedCmd_NoOperation,
iBufferedCmd_MoveAbsolute,
iBufferedCmd_MoveRelative,
iBufferedCmd_MoveVelocity,
(**)
iBufferedCmd_Stop,
iBufferedCmd_ResetAndStop,
iBufferedCmd_Halt,
iBufferedCmd_CamIn,
iBufferedCmd_GearIn,
iBufferedCmd_Power,
iBufferedCmd_Home,
iBufferedCmd_StepAbsSwitch,
iBufferedCmd_StepLimitSwitch,
iBufferedCmd_StepBlock,
iBufferedCmd_StepDirect,
iBufferedCmd_FinishHoming,
(**)
iBufferedCmdEx_Jerk:=100,
iBufferedCmdEx_Acc,
iBufferedCmdEx_Velo,
iBufferedCmdEx_Creep,
(**)
iBufferedCmd_
);
END_TYPE

TF5810

Version: 1.8.3

89

PLCopen Motion Control

Values

Name
iBufferedCmd_NoOperation

iBufferedCmd_MoveAbsolute

iBufferedCmd_MoveRelative

iBufferedCmd_MoveVelocity

iBufferedCmd_Stop
iBufferedCmd_ResetAndStop
iBufferedCmd_Halt
iBufferedCmd_CamIn
iBufferedCmd_GearIn
iBufferedCmd_Power
iBufferedCmd_Home
iBufferedCmd_StepAbsSwitch
iBufferedCmd_StepLimitSwitch
iBufferedCmd_StepBlock
iBufferedCmd_StepDirect
iBufferedCmd_FinishHoming
iBufferedCmdEx_Jerk

iBufferedCmdEx_Acc

iBufferedCmdEx_Velo

iBufferedCmdEx_Creep

Description
This constant is used as initial value for call parameters of function
blocks and in variables.
The buffered command was entered by an
MC_MoveAbsolute_BkPlcMc function block. See note #1.
The buffered command was entered by an
MC_MoveRelative_BkPlcMc function block. See note #1.
The buffered command was entered by an
MC_MoveVelocity_BkPlcMc function block. See note #1.
reserved, not implemented.
reserved, not implemented.
reserved, not implemented.
reserved, not implemented.
reserved, not implemented.
reserved, not implemented.
reserved, not implemented.
reserved, not implemented.
reserved, not implemented.
reserved, not implemented.
reserved, not implemented.
reserved, not implemented.
The command component associated with constant-jerk motion was
entered by a function block. See note #2.
The command component associated with constant acceleration or
deceleration was entered by a function block. See note #2.
The command component associated with constant-velocity motion
was entered by a function block. See note #2.
reserved, not implemented.

#1: If the axis uses a setpoint generator type without Look Ahead, complete commands are entered
as a buffer element.

#2: If the axis uses a setpoint generator type with Look Ahead, commands are split into sections
and entered as a package typically consisting of seven buffer elements (jerk, acceleration, jerk,
velocity, jerk, deceleration, jerk).

4.3.3

E_TcMcCurrentStep

Available from version 3.0

The constants in this list are used for identifying the internal states of the control value generators.

Not all of these states are used by all control value generators.

Syntax
TYPE E_TcMcCurrentStep :(
iTcHydStateIdle,
iTcHydStateTcAccP,

90

Version: 1.8.3

TF5810

PLCopen Motion Control

iTcHydStateTcAccM,
iTcHydStatePcAccP,
iTcHydStatePcAccM,
iTcHydStateConstVeloP,
iTcHydStateConstVeloM,
iTcHydStatePcDecP,
iTcHydStatePcDecM,
iTcHydStateCreepVeloP,
iTcHydStateCreepVeloM,
iTcHydStateTcDecP,
iTcHydStateTcDecM,
iTcHydStateFeedStopPos,
iTcHydStateFeedStopNeg,
iTcHydStateDoBrake,
iTcHydStateCoupling := 1000,
iTcHydStateCoupled,
iTcHydStateExtCoupled,
iTcHydStateExtGenerated := 2000,
iTcHydStateEmergencyBreak := 9000,
iTcHydStateFault := 9999
);
END_TYPE

TF5810

Version: 1.8.3

91

PLCopen Motion Control

Values

92

Version: 1.8.3

TF5810

Name
iTcHydStateIdle

iTcHydStateTcAccP

iTcHydStateTcAccM

iTcHydStatePcAccP

iTcHydStatePcAccM

iTcHydStateConstVeloP

iTcHydStateConstVeloM

iTcHydStatePcDecP

iTcHydStatePcDecM

iTcHydStateCreepVeloP

iTcHydStateCreepVeloM

iTcHydStateTcDecP

iTcHydStateTcDecM

iTcHydStateFeedStopPos

PLCopen Motion Control

Description
The axis is not actively moving. Its behavior is controlled by
ST_TcHydAxParam.fLagAmp, ST_TcHydAxParam.fTargetClamping
and ST_TcHydAxParam.fReposDistance.
The axis establishes a positive control value according to
ST_TcHydAxRtData.fDestAcc. This value is set by one of the start
function blocks according to the data of the travel command. This state
is assumed when the control value reaches the specified motion
control value. If the system detects that the braking process for the
target approach has to be initiated, the state is changed to
iTcHydStatePcDecP. In the absence of feed enable, the state is
changed to iTcHydStateFeedStopPos.
The axis establishes a negative control value according to
ST_TcHydAxRtData.fDestAcc. This value is set by one of the start
function blocks according to the data of the travel command. This state
is assumed when the control value reaches the specified motion
control value. If the system detects that the braking process for the
target approach has to be initiated, the state is changed to
iTcHydStatePcDecM. In the absence of feed enable, the state is
changed to iTcHydStateFeedStopNeg.
The axis is in the displacement-controlled acceleration phase of a
travelling motion in positive direction. The control value is set to a value
specified by the travel command according to
ST_TcHydAxRtData.fDestAcc. The state then changes to
iTcHydStateConstVeloP.
The axis is in the displacement-controlled acceleration phase of a
travelling motion in negative direction. The control value is set to a
value specified by the travel command according to
ST_TcHydAxRtData.fDestAcc. The state then changes to
iTcHydStateConstVeloM.
The axis travels in positive direction with constant control value. The
control value is specified by the travel command.
The axis travels in negative direction with constant control value. The
control value is specified by the travel command.
The axis is in the displacement-controlled brake phase of a travelling
motion in positive direction. The control value is reduced to
ST_TcHydAxParam.fCreepSpeed. The state then changes to
iTcHydStateCreepVeloP.
The axis is in the displacement-controlled brake phase of a travelling
motion in negative direction. The control value is reduced to
ST_TcHydAxParam.fCreepSpeed. The state then changes to
iTcHydStateCreepVeloM.
The axis travels in positive direction with constant control value. The
control value is specified by ST_TcHydAxParam.fCreepSpeed.
The axis travels in negative direction with constant control value. The
control value is specified by ST_TcHydAxParam.fCreepSpeed.
The axis executes a regular stop, starting from a travelling motion in
positive direction. The control value is reduced with
ST_TcHydAxParam.fStopRamp. The state then changes to
iTcHydStateIdle.
The axis executes a regular stop, starting from a travelling motion in
negative direction. The control value is reduced with
ST_TcHydAxParam.fStopRamp. The state then changes to
iTcHydStateIdle.
The axis executes an intermediate stop, due to lack of feed enable in
positive direction (dwTcHydDcDwFdPosEna is not set in
ST_TcHydAxRtData.nDeCtrlDWord). The control value is reduced with
ST_TcHydAxParam.fStopRamp. The axis then waits for a feed enable.

TF5810

Version: 1.8.3

93

PLCopen Motion Control

Name
iTcHydStateFeedStopNeg

iTcHydStateDoBrake

iTcHydStateCoupling
iTcHydStateCoupled

iTcHydStateExtCoupled

iTcHydStateExtGenerated

iTcHydStateEmergencyBreak

iTcHydStateFault

Description
The axis executes an intermediate stop, due to lack of feed enable in
negative direction (dwTcHydDcDwFdNegEna is not set in
ST_TcHydAxRtData.nDeCtrlDWord). The control value is reduced with
ST_TcHydAxParam.fStopRamp. The axis then waits for a feed enable.
The axis executes a waiting time. This is necessary, if switching is
required due to a brake or a switching valve.
The axis is in transition to state iTcHydStateCoupled.
The control value of the axis is derived from the control value of
another axis based on the principle of electronic gearing.
The control value of the axis is calculated based on the principle of
continuously variable transmission.
The control value of the axis is generated by an external function block.
This may be a library function block or an application-specific function
block.
The axis performs an emergency stop. The control value is reduced
with ST_TcHydAxParam.fEmergencyRamp. The system then checks
whether the axis is in an error state (ST_TcHydAxRtData.nErrorCode
not equal 0). If yes, the state is changed to iTcHydStateFault,
otherwise iTcHydStateIdle.
The axis is in an error state. It does not carry out actively control
movements and does not accept motion commands. To put the axis
back in an undisturbed state, call a function block of type
MC_Reset_BkPlcMc or MC_ResetAndStop_BkPlcMc.

4.3.4

E_TcMcDriveType

Available from version 3.0

The constants listed here are used to identify the hardware used to output the control values for an axis.

Syntax
TYPE E_TcMcDriveType :(
(*
The sequence below must not be changed!
New types have to be added at the end.
In case a type becomes obsolete it has to be replaced by a dummy
to ensure the numerical meaning of the other codes.
*)
(*
Die bestehende Reihenfolge darf nicht veraendert werden.
Neue Typen muessen am Ende eingefuegt werden.
Wenn ein Typ wegfallen sollte, muss er durch einen Dummy
ersetzt werden, um die numerische Zuordnung zu garantieren.
*)
(* last modification: 26.02.2016 *)
iTcMc_Drive_Customized,
iTcMc_DriveLowCostStepper,
iTcMc_DriveKL2521,
iTcMc_DriveKL4032,
iTcMc_DriveAx2000_B900R,
iTcMc_DriveM2400_D1,
iTcMc_DriveM2400_D2,
iTcMc_DriveM2400_D3,
iTcMc_DriveM2400_D4,
iTcMc_DriveLowCostStepperHS,
iTcMc_DriveLowCostStepperFS,
iTcMc_DriveIx2512_1Coil,
iTcMc_DriveIx2512_2Coil,
iTcMc_DriveKL2531,
iTcMc_DriveKL2541,
iTcMc_DriveEL4132,
iTcMc_DriveAx2000_B200R,

94

Version: 1.8.3

TF5810

PLCopen Motion Control

iTcMc_DriveAx2000_B110R,
iTcMc_DriveKL2532,
iTcMc_DriveKL2552,
iTcMc_DriveKL2535_1Coil,
iTcMc_DriveKL2535_2Coil,
iTcMc_DriveKL2545_1Coil,
iTcMc_DriveKL2545_2Coil,
iTcMc_DriveLowCostInverter,
iTcMc_Drive_CoE_DS408,
iTcMc_DriveAx2000_B110A,
iTcMc_DriveAx5000_B110A,
iTcMc_DriveAx2000_B750A,
iTcMc_Drive_CoE_DS402,
iTcMc_DriveAx5000_B110SR,
iTcMc_DriveEL4x22,
iTcMc_DriveEL2521,
iTcMc_DrivePumpEtcIO,
iTcMc_DriveEL2535_1Coil,
iTcMc_DriveEL2535_2Coil,
iTcMc_DriveEL7201,
iTcMc_DriveEL7037,
iTcMc_DriveEL7047,
iTcMc_DriveEM8908,
iTcMc_DriveAx5000_B110INC,
iTcMc_Drive_TestOnly:=1000
);
END_TYPE

TF5810

Version: 1.8.3

95

PLCopen Motion Control

Values

96

Version: 1.8.3

TF5810

Name
iTcMc_Drive_Customized

iTcMc_DriveLowCostStepper

iTcMc_DriveKL2521

iTcMc_DriveKL4032

iTcMc_DriveAx2000_B900R

iTcMc_DriveM2400_D1

iTcMc_DriveM2400_D2

iTcMc_DriveM2400_D3

iTcMc_DriveM2400_D4

iTcMc_DriveLowCostStepperHS

iTcMc_DriveLowCostStepperFS

iTcMc_DriveIx2512_1Coil

iTcMc_DriveIx2512_2Coil

iTcMc_DriveKL2531

iTcMc_DriveKL2541

iTcMc_DriveEL4132

iTcMc_DriveAx2000_B200R

iTcMc_DriveAx2000_B110R

iTcMc_DriveKL2532

PLCopen Motion Control

Description
iTcMc_Drive_Customized: The control value for the drive has not
been prepared for output on any particular hardware. This process
must be carried out by the PLC application itself.
iTcMc_DriveLowCostStepper: The incremental set position changes
are generated as a digital output signals for a directly controlled
stepper motor. This code continues to be supported for reasons of
compatibility, and has the same meaning as
iTcMc_DriveLowCostStepperHS.
iTcMc_DriveKL2521: The control value for the drive has been
appropriately processed for output on a KL2521 Pulse Train
terminal.
iTcMc_DriveKL4032: The control value for the drive has been
appropriately processed for output on a ±10 V KL4032 analog
output terminal.
iTcMc_DriveAx2000_B900R: The control value for the drive is
processed for output on an AX2000 servo drive at a resolver motor
at the Beckhoff RealTime Ethernet fieldbus.
iTcMc_DriveM2400_D1: The control value for the drive has been
appropriately processed for output on the first channel of an M2400
Box on the Beckhoff II/O.
iTcMc_DriveM2400_D2: The control value for the drive has been
appropriately processed for output on the second channel of an
M2400 Box on the Beckhoff II/O.
iTcMc_DriveM2400_D3: The control value for the drive has been
appropriately processed for output on the third channel of an M2400
Box on the Beckhoff II/O.
iTcMc_DriveM2400_D4: The control value for the drive has been
appropriately processed for output on the fourth channel of an
M2400 Box on the Beckhoff II/O.
iTcMc_DriveLowCostStepperHS: The incremental set position
changes are generated as a digital output signals for a directly
controlled stepper motor. Half-stepping is being used.
iTcMc_DriveLowCostStepperFS: The incremental set position
changes are generated as a digital output signals for a directly
controlled stepper motor. Full-stepping is being used.
iTcMc_DriveIx2512_1Coil: The control value for the drive is
processed for output on a Fieldbus Box IP/IE2512. The rules for
valves with one coil apply.
iTcMc_DriveIx2512_2Coil: The control value for the drive is
processed for output on a Fieldbus Box IP/IE2512. The rules for
valves with two coils apply.
iTcMc_DriveKL2531: The control value for the drive is processed for
output on a KL2531 stepper motor output stage terminal.
iTcMc_DriveKL2541: The control value for the drive is processed for
output on a KL2541 stepper motor output stage terminal.
iTcMc_DriveEL4132: The control value for the drive has been
appropriately processed for output on a ±10 V EL4132 analog
output terminal.
iTcMc_DriveAx2000_B200R: The control value for the drive is
processed for output on an AX2000 servo drive at a resolver motor
at the Beckhoff II/O fieldbus.
iTcMc_DriveAx2000_B110R: The control value for the drive is
processed for output on an AX2000 servo drive at a resolver motor
at the EtherCAT fieldbus.
iTcMc_DriveKL2532: The control value for the drive is processed for
output on a KL2532 DC motor output stage terminal.

TF5810

Version: 1.8.3

97

PLCopen Motion Control

Name
iTcMc_DriveKL2552
iTcMc_DriveKL2535_1Coil

iTcMc_DriveKL2535_2Coil

iTcMc_DriveKL2545_1Coil

iTcMc_DriveKL2545_2Coil

iTcMc_DriveLowCostInverter

iTcMc_Drive_CoE_DS408

iTcMc_DriveAx2000_B110A

iTcMc_DriveAx5000_B110A

iTcMc_DriveAx2000_B750A

iTcMc_Drive_CoE_DS402
iTcMc_DriveAx5000_B110SR

iTcMc_DriveEL4x22
iTcMc_DriveEL2521

iTcMc_DrivePumpEtcIO
iTcMc_DriveEL2535_1Coil

iTcMc_DriveEL2535_2Coil

iTcMc_DriveEL7201

iTcMc_DriveEL7037

iTcMc_DriveEL7047

iTcMc_DriveEM8908
iTcMc_DriveAx5000_B110INC

iTcMc_Drive_TestOnly

Description

iTcMc_DriveKL2535_1Coil: The control value for the drive is
processed for output on a KL2535 PMW output stage terminal.
iTcMc_DriveKL2535_2Coil: The control value for the drive is
processed for output on a KL2535 PMW output stage terminal.
iTcMc_DriveKL2545_1Coil: The control value for the drive is
processed for output on a KL2545 PMW output stage terminal.
iTcMc_DriveKL2545_2Coil: The control value for the drive is
processed for output on a KL2545 PMW output stage terminal.
iTcMc_DriveLowCostInverter: The control value for the drive is
processed for output in the form of digital output signals for a
frequency converter with programmed fixed frequencies.
iTcMc_Drive_CoE_DS408: The control value for the drive is
processed for output on a proportional valve at the EtherCAT
fieldbus.
iTcMc_DriveAx2000_B110A: The control value for the drive is
processed for output on an AX2000 servo drive at an absolute multi-
turn encoder motor at the EtherCAT fieldbus.
iTcMc_DriveAx5000_B110A: The control value for the drive is
processed for output on an AX5000 servo drive at an absolute multi-
turn encoder motor at the EtherCAT fieldbus.
iTcMc_DriveAx2000_B750A: The control value for the drive is
processed for output on an AX2000 servo drive at an absolute multi-
turn encoder motor at the Sercos fieldbus.
iTcMc_Drive_CoE_DS402: In preparation.
iTcMc_DriveAx5000_B110SR: The control value for the drive is
processed for output on an AX5000 servo drive at an absolute
single-turn encoder or resolver motor at the EtherCAT fieldbus.
iTcMc_DriveEL4x22: In preparation.
iTcMc_DriveEL2521: The control value for the drive has been
appropriately processed for output on a KL2521 Pulse Train
terminal.
iTcMc_DrivePumpEtcIO: Reserved
iTcMc_DriveEL2535_1Coil: The control value for the drive is
processed for output on an EL2535 PMW output stage terminal.
iTcMc_DriveEL2535_2Coil: The control value for the drive is
processed for output on an EL2535 PMW output stage terminal.
iTcMc_DriveEL7201: The control value for the drive has been
appropriately processed for output on a EL7201 servo terminal.
iTcMc_DriveEL7037: The control value for the drive is processed for
output on an EL7037 stepper motor output stage terminal.
iTcMc_DriveEL7047: The control value for the drive is processed for
output on an EL7047 stepper motor output stage terminal.
iTcMc_DriveEM8908: Reserved for sector-specific package.
iTcMc_DriveAx5000_B110INC: The control value for the drive is
processed for output on an AX5000 servo drive at an incremental
encoder at the EtherCAT fieldbus.
iTcMc_Drive_TestOnly: Reserved for internal testing; do not use.

4.3.5

E_TcMcEncoderType

Available from version 3.0

98

Version: 1.8.3

TF5810

The constants listed here are used to identify the hardware used to acquire the actual values for an axis.

PLCopen Motion Control

Syntax
TYPE E_TcMcEncoderType :(
(*
The sequence below must not be changed!
New types have to be added at the end.
In case a type becomes obsolete it has to be replaced by a dummy
to ensure the numerical meaning of the other codes.
*)
(*
Die bestehende Reihenfolge darf nicht veraendert werden.
Neue Typen muessen am Ende eingefuegt werden.
Wenn ein Typ wegfallen sollte, muss er durch einen Dummy
ersetzt werden, um die numerische Zuordnung zu garantieren.
*)
(* last modification: 17.01.2013 *)
iTcMc_EncoderSim,
iTcMc_EncoderDigIncrement,
iTcMc_EncoderLowCostStepper,
iTcMc_EncoderKL2521,
iTcMc_EncoderKL3042,
iTcMc_EncoderKL5001,
iTcMc_EncoderKL5101,
iTcMc_EncoderAx2000_B900R,
iTcMc_EncoderDigCam,
iTcMc_EncoderIx5009,
iTcMc_EncoderM2510,
iTcMc_EncoderKL3002,
iTcMc_EncoderKL2531,
iTcMc_EncoderKL5111,
iTcMc_EncoderAbs32,
iTcMc_EncoderM3120,
iTcMc_EncoderKL2541,
iTcMc_EncoderEL3102,
iTcMc_EncoderEL3142,
iTcMc_EncoderEL5001,
iTcMc_EncoderEL5101,
iTcMc_EncoderEL5111,
iTcMc_EncoderKL3062,
iTcMc_EncoderKL3162,
iTcMc_EncoderAx2000_B200R,
iTcMc_EncoderAx2000_B110R,
iTcMc_EncoderEL3162,
iTcMc_EncoderKL2542,
iTcMc_EncoderKL2545,
iTcMc_EncoderAx2000_B110A,
iTcMc_EncoderAx5000_B110A,
iTcMc_EncoderAx2000_B750A,
iTcMc_EncoderCoE_DS406,
iTcMc_EncoderCoE_DS402SR,
iTcMc_EncoderAx5000_B110SR,
iTcMc_EncoderCoE_DS402A,
iTcMc_EncoderEL2521,
iTcMc_EncoderAbs32Etc,
iTcMc_EncoderEL7201SR,
iTcMc_EncoderDigPulseCount,
iTcMc_EncoderEL3255,
iTcMc_EncoderEL7047,
iTcMc_DriveEM8908A,
iTcMc_DriveEM8908C,
iTcMc_EncoderCoE5001,
iTcMc_EncoderEL7201A,
iTcMc_DriveAx5000_B110INC,
iTcMc_EncoderEL5032,
iTcMc_EncoderEL5021,
iTcMc_Encoder_TestOnly:=1000
);
END_TYPE

TF5810

Version: 1.8.3

99

PLCopen Motion Control

Values

100

Version: 1.8.3

TF5810

Name
iTcMc_EncoderSim

Description
The virtual actual position of the axis is a copy of the set position.

PLCopen Motion Control

iTcMc_EncoderDigIncrement

iTcMc_EncoderLowCostStepper

iTcMc_EncoderKL2521

iTcMc_EncoderKL3042

iTcMc_EncoderKL5001

iTcMc_EncoderKL5101

iTcMc_EncoderAx2000_B900R

iTcMc_EncoderDigCam
iTcMc_EncoderIx5009

iTcMc_EncoderM2510

iTcMc_EncoderKL3002

iTcMc_EncoderKL2531

iTcMc_EncoderKL5111

iTcMc_EncoderAbs32

iTcMc_EncoderM3120

iTcMc_EncoderKL2541

iTcMc_EncoderEL3102

iTcMc_EncoderEL3142

iTcMc_EncoderEL5001

iTcMc_EncoderEL5101

iTcMc_EncoderEL5111

iTcMc_EncoderKL3062

On a real machine this type must only be used for virtual
axes. Otherwise the axis will carry out uncontrolled and
unpredictable movements.
The incremental actual value of the axis is generated by
evaluating two digital input bits. These represent the A and B
tracks of an incremental encoder, and are evaluated, in
accordance with the principle of a quadrature decoder, using
quadruple evaluation.

Only one of the input bits may change its state in each PLC
cycle. This means that the maximum velocity is one
increment per TCycle.
Incremental changes to the actual position are generated from the
output signals for a digitally operated stepper motor.
The incremental actual position is generated from the pulse
counter of a KL2521 Pulse Train terminal.
The absolute actual position is generated from the ADW value of a
0..20 mA KL3042 analog input terminal.
The absolute actual position is generated from the counter value
from a KL5001 SSI input terminal.
The incremental actual position is generated from the counter
value from a KL5101 input terminal.
The incremental actual position is determined from the counter
value of an AX2000 servo drive at a resolver motor at the Beckhoff
RealTime Ethernet fieldbus.
The position cam byte is generated from four digital input bits.
The absolute actual position is generated from the counter value
of an SSI IP/IE5009 Fieldbus Box.
The absolute actual position is generated from the ADW value of a
±10 V M2510 analog input box.
The absolute actual position is generated from the ADW value of a
±10 V KL3002 analog input terminal.
The incremental actual position is generated from the pulse
counter of a KL2531 stepper motor terminal.
The incremental actual position is generated from the counter
value from a KL5111 input terminal.
The absolute actual position is generated from the 32-bit value of
a general electronic input system.
The incremental actual position is generated from the counter
value of an M3120 Lightbus module.
The incremental actual position is generated from the pulse
counter (motor pulse or encoder) of a KL2541 stepper motor
terminal.
The absolute actual position is generated from the ADW value of a
±10 V EL3102 analog input terminal.
The absolute actual position is generated from the ADW value of a
0..20 mA EL3142 analog input terminal.
The absolute actual position is generated from the counter value
from an EL5001 SSI input terminal.
The incremental actual position is generated from the counter
value from an EL5101 input terminal.
The incremental actual position is generated from the counter
value from an EL5111 input terminal.
The absolute actual position is generated from the ADW value of a
0..10 V KL3062 analog input terminal.

TF5810

Version: 1.8.3

101

PLCopen Motion Control

Name
iTcMc_EncoderKL3162

iTcMc_EncoderAx2000_B200R

iTcMc_EncoderAx2000_B110R

iTcMc_EncoderEL3162

iTcMc_EncoderKL2542

iTcMc_EncoderKL2545

iTcMc_EncoderAx2000_B110A

iTcMc_EncoderAx5000_B110A

iTcMc_EncoderAx2000_B750A

iTcMc_EncoderCoE_DS406
iTcMc_EncoderCoE_DS402SR
iTcMc_EncoderAx5000_B110SR

iTcMc_EncoderCoE_DS402A
iTcMc_EncoderEL2521

iTcMc_EncoderAbs32Etc

iTcMc_EncoderEL7201SR

iTcMc_EncoderDigPulseCount

iTcMc_EncoderEL3255
iTcMc_EncoderEL7047
iTcMc_DriveEM8908A
iTcMc_DriveEM8908C
iTcMc_EncoderCoE5001
iTcMc_EncoderEL7201A
iTcMc_DriveAx5000_B110INC

iTcMc_EncoderEL5032

iTcMc_EncoderEL5021

iTcMc_Encoder_TestOnly

Description
The absolute actual position is generated from the ADW value of a
0..10 V KL3162 analog input terminal.
The incremental actual position is determined from the counter
value of an AX2000 actuator at a resolver motor at the Beckhoff II/
O fieldbus.
The incremental actual position is determined from the counter
value of an AX2000 actuator at a resolver motor at the EtherCAT
fieldbus.
The absolute actual position is generated from the ADW value of a
0..10 V EL3162 analog input terminal.
The incremental actual position is generated from the counter
value from a KL2542 input terminal.
The incremental actual position is generated from the counter
value from a KL2545 input terminal.
The absolute actual position is determined from the counter value
of an AX2000 actuator at an absolute multi-turn encoder motor at
the EtherCAT fieldbus.
The absolute actual position is determined from the counter value
of an AX5000 actuator at an absolute multi-turn encoder motor at
the EtherCAT fieldbus.
The absolute actual position is determined from the counter value
of an AX2000 actuator at an absolute multi-turn encoder motor at
the Sercos fieldbus.
An encoder with CoE_406 support at the EtherCAT fieldbus.
In preparation.
The incremental actual position is determined from the counter
value of an AX5000 actuator at a single turn encoder motor at the
EtherCAT fieldbus.
In preparation.
The incremental actual position is generated from the pulse
counter of an EL2521 Pulse Train terminal.
The absolute actual position is generated from the 32-bit value of
a general EtherCAT electronic input system. Profile support is not
a precondition.
The incremental actual position is generated from the counter
value from an EL7201 servo terminal.
Counts the edges (positive and negative) of pulses. The direction
of rotation is determined via the drive output.

Only one pulse can be detected per PLC cycle.
iTcMc_EncoderEL3255: In preparation.

The incremental actual position is determined from the counter
value of an AX5000 actuator at an incremental encoder at the
EtherCAT fieldbus.
The absolute actual position is generated from the counter value
of an EL5032 EnDat-2.2 input terminal.
The absolute actual position is generated from the counter value
of an EL5021 sin/cos input terminal.
Reserved for internal testing; do not use.

102

Version: 1.8.3

TF5810

PLCopen Motion Control

4.3.6

E_TcMCFbState

Available from version 3.0

The constants listed here are used to identify the runtime states of the axes.

Syntax
TYPE E_TcMCFbState :
(
McState_Standstill := 0,
McState_DiscreteMotion,
McState_Continousmotion,
McState_Synchronizedmotion,
McState_Stopping,
McState_Errorstop,
McState_Homing,
McState_Disabled
);
END_TYPE

Values

Name
McState_Standstill

McState_DiscreteMotion

McState_Continousmotion

McState_Synchronizedmotion

McState_Stopping
McState_Errorstop

McState_Homing
McState_Disabled

Description
The axis does not have a motion command. Active position control,
repositioner monitoring, the output of a press control value or none of
these will be carried out, depending on the parameterization.
The axis executes a movement with a defined target position and
velocity.
The axis executes a movement without any defined target position.
Only the velocity is specified.
The axis performs a movement, which is derived from the movement
of another axis.
The axis is carrying out a stop.
The axis has been stopped because of a problem. It cannot at
present be started, and requires a reset before it will be in a condition
from which it can start.
The axis is homing.
The controller enable of the axis is FALSE.

4.3.7

E_TcMcHomingType

Available from version 3.0

The constants listed here are used to identify the referencing method of the axes.

Syntax
TYPE E_TcMcHomingType :(
iTcMc_HomingOnBlock,
iTcMc_HomingOnIndex,
iTcMc_HomingOnSync,
iTcMc_HomingOnMultiSync,
iTcMc_HomingOnExec
);
END_TYPE

TF5810

Version: 1.8.3

103

PLCopen Motion Control

Values

Name
iTcMc_HomingOnBlock

iTcMc_HomingOnIndex

iTcMc_HomingOnSync

iTcMc_HomingOnMultiSync

iTcMc_HomingOnExec

Description
The axis is moved in the direction specified by
ST_TcHydAxParam.bEnc_RefIndexPositive with
ST_TcHydAxParam.fEnc_RefIndexVelo. If no movement is detected
over a period of 2 seconds, the fixed stop (block) is considered to have
been reached. The actual value for the axis is set to the value of the
reference position.
The axis is moved in the direction specified by
ST_TcHydAxParam.bEnc_RefIndexPositive with
ST_TcHydAxParam.fEnc_RefIndexVelo. The axis is stopped if the
referencing cam (bit 5, dwTcHydDcDwRefIndex) is detected in
ST_TcHydAxRtData.nDeCtrlDWord. It is then moved with
ST_TcHydAxParam.fEnc_RefSyncVelo in the direction specified by
ST_TcHydAxParam.bEnc_RefSyncPositive until the referencing cam
has again been left. The actual value for the axis is set to the value of
the reference position.
The axis is moved in the direction specified by
ST_TcHydAxParam.bEnc_RefIndexPositive with
ST_TcHydAxParam.fEnc_RefIndexVelo. The axis is stopped if the
referencing cam (bit 5, dwTcHydDcDwRefIndex) is detected in
ST_TcHydAxRtData.nDeCtrlDWord. It is then moved with
ST_TcHydAxParam.fEnc_RefSyncVelo in the direction specified by
ST_TcHydAxParam.bEnc_RefSyncPositive until the referencing cam
has again been left. The encoder's hardware latch is then activated, and
the axis is moved on until the latch becomes valid. After the axis has
stopped, the actual value for the axis is set to a value that is calculated
from the reference position and from the distance covered since the
encoder's sync pulse was detected.
The actual value of the axis is immediately set to the value of the
reference position.
The hardware latch of the encoder is activated. The axis is moved with
ST_TcHydAxParam.fEnc_RefSyncVelo in the direction specified by
ST_TcHydAxParam.bEnc_RefIndexPositive, until the latch has become
valid twice. If the end of path is detected before two sync pulses were
detected, the process is repeated in the opposite direction. If this does
not succeed either, the homing is aborted. Otherwise the current actual
position is determined from the distance of the sync pulses and the
fEnc_BaseDistance.

4.3.8

E_TcMCParameter

Available from version 3.0

The constants listed here are used for numbering parameters.

Syntax
TYPE E_TcMCParameter :
(
(*
==================================================================
A T T E N T I O N
==================================================================
= These Codes are also used to identify parameters in files.
= Any change of the meaning of any code here will make any file
= incompatible without notice and may even cause a crash of
= the control system!
==================================================================
= CONSEQUENCE: Only adding new codes is allowed!

104

Version: 1.8.3

TF5810

PLCopen Motion Control

==================================================================
= These codes are also used for ADS communication
==================================================================
*)
(*
==================================================================
A C H T U N G
==================================================================
= Diese Codes werden auch zur Kennzeichnung von Parametern
= in den Dateien verwendet. Eine Veraenderung der Codes wuerde
= die Dateien (nicht erkennbar) inkompatibel machen und koennte
= zum Systemabsturz fuehren!
==================================================================
= ALSO: Es duerfen nur neue Codes dazugefuegt werden!
==================================================================
= Diese Codes werden ebenfalls fuer die ADS-Kommunikation benutzt
==================================================================
*)
McPara_CommandedPosition:=1,
McPara_SWLimitPos,
McPara_SWLimitNeg,
McPara_EnableLimitPos,
McPara_EnableLimitNeg,
McPara_EnablePosLagMonitoring,
McPara_MaxPositionLag,
McPara_MaxVelocitySystem,
McPara_MaxVelocityAppl,
McPara_ActualVelocity,
McPara_CommandedVelocity,
McPara_MaxAccelerationSystem,
McPara_MaxAccelerationAppl,
McPara_MaxDecelerationSystem,
McPara_MaxDecelerationAppl,
McPara_MaxJerk,
(* ============================================================ *)
McPara_BkPlcMc_ProfilType:=1000,
McPara_BkPlcMc_EnvCycletime,
McPara_BkPlcMc_AxName,
McPara_BkPlcMc_TimeBased,
McPara_BkPlcMc_JerkEnabled,
McPara_BkPlcMc_LogLevel,
McPara_BkPlcMc_CycleDivider,
McPara_BkPlcMc_ParamFileName,

McPara_BkPlcMc_EncoderType:=1100,
McPara_BkPlcMc_EncoderHomingType,
McPara_BkPlcMc_EncoderZeroShift,
McPara_BkPlcMc_EncoderIncWeighting,
McPara_BkPlcMc_EncoderIncInterpolation,
McPara_BkPlcMc_EncoderRefIndexVelo,
McPara_BkPlcMc_EncoderRefIndexPositive,
McPara_BkPlcMc_EncoderRefSyncVelo,
McPara_BkPlcMc_EncoderRefSyncPositive,
McPara_BkPlcMc_EncoderDefaultHomePosition,
McPara_BkPlcMc_EncoderReversed,
McPara_BkPlcMc_EncoderBaseDistance,
McPara_BkPlcMc_EncoderModuloBase,
McPara_BkPlcMc_EncoderEnableLatch,
McPara_BkPlcMc_EncoderLatchedPos,
McPara_BkPlcMc_EncoderRefShift,
McPara_BkPlcMc_EncoderRefFlag,
McPara_BkPlcMc_EncoderPotiRgToRl,
McPara_BkPlcMc_EncoderOverrunMask,
McPara_BkPlcMc_EncoderPositionMask,
McPara_BkPlcMc_EncoderZeroSwap,
McPara_BkPlcMc_EncoderNoUpload,
McPara_BkPlcMc_EncoderModuloMode,

McPara_BkPlcMc_ValveOverlapCompP:=1200,
McPara_BkPlcMc_ValveBendPointVelo,
McPara_BkPlcMc_ValveBendPointOutput,
McPara_BkPlcMc_ValveResponseTime,
McPara_BkPlcMc_ValveOverlapCompM,
McPara_BkPlcMc_CylinderArreaA:=1280,
McPara_BkPlcMc_CylinderArreaB,

McPara_BkPlcMc_DriveType:=1300,
McPara_BkPlcMc_AreaRatio,
McPara_BkPlcMc_DriveReversed,
McPara_BkPlcMc_DriveDefaultPowerOk

TF5810

Version: 1.8.3

105

PLCopen Motion Control

McPara_BkPlcMc_DriveAbsoluteOutput,
McPara_BkPlcMc_DriveIncWeighting,
McPara_BkPlcMc_DriveIncInterpolation,
McPara_BkPlcMc_DriveNoUpload,

McPara_BkPlcMc_DriveIsHybrid,
McPara_BkPlcMc_HybridConcept,
McPara_BkPlcMc_Pump_Cavities,
McPara_BkPlcMc_Pump_EncType,
McPara_BkPlcMc_Pump_N_max,
McPara_BkPlcMc_Pump_N_min,
McPara_BkPlcMc_Pump_P_max,
McPara_BkPlcMc_Pump_P_min,
McPara_BkPlcMc_Pump_Q_fast_P,
McPara_BkPlcMc_Pump_Q_slow_P,
McPara_BkPlcMc_Pump_Q_fast_M,
McPara_BkPlcMc_Pump_Q_slow_M,
McPara_BkPlcMc_Pump_Q_leak,
McPara_BkPlcMc_Pump_Enc_Offset,
McPara_BkPlcMc_Cylinder_A_addP,
McPara_BkPlcMc_Cylinder_A_addM,
McPara_BkPlcMc_PrsScaling_A,
McPara_BkPlcMc_PrsScaling_B,
McPara_BkPlcMc_PrsScaling_Sys,
McPara_BkPlcMc_Motor_RampTime,
McPara_BkPlcMc_Pump_Regenerative,
McPara_BkPlcMc_Virtual_A_addP,
McPara_BkPlcMc_Virtual_A_addM,
McPara_BkPlcMc_Aside_PrsHiResADC,
McPara_BkPlcMc_Bside_PrsHiResADC,
McPara_BkPlcMc_System_PrsHiResADC,

McPara_BkPlcMc_StartRamp:=1400,
McPara_BkPlcMc_obsolete_1,
McPara_BkPlcMc_obsolete_2,

McPara_BkPlcMc_StopRamp:=1500,
McPara_BkPlcMc_EmergencyRamp,
McPara_BkPlcMc_BrakeOn,
McPara_BkPlcMc_BrakeOff,
McPara_BkPlcMc_BrakeSafety,

McPara_BkPlcMc_CreepSpeedP:=1600,
McPara_BkPlcMc_CreepDistanceP,
McPara_BkPlcMc_BrakeDistanceP,
McPara_BkPlcMc_BrakeDeadTimeP,
McPara_BkPlcMc_CreepSpeedM,
McPara_BkPlcMc_CreepDistanceM,
McPara_BkPlcMc_BrakeDistanceM,
McPara_BkPlcMc_BrakeDeadTimeM,
McPara_BkPlcMc_AsymetricalTargeting,

McPara_BkPlcMc_LagAmp:=1700,
McPara_BkPlcMc_LagAmpAdaptLimit,
McPara_BkPlcMc_LagAmpAdaptFactor,
McPara_BkPlcMc_ZeroCompensation,
McPara_BkPlcMc_TargetClamping,
McPara_BkPlcMc_ReposDistance,
McPara_BkPlcMc_AutoBrakeDistance,
McPara_BkPlcMc_EnableControlLoopOnFault,
McPara_BkPlcMc_LagAmpDx,
McPara_BkPlcMc_LagAmpTi,
McPara_BkPlcMc_LagAmpWuLimit,
McPara_BkPlcMc_LagAmpOutLimit,

McPara_BkPlcMc_VeloAmp,
McPara_BkPlcMc_VeloAmpDx,
McPara_BkPlcMc_VeloAmpTi,
McPara_BkPlcMc_VeloAmpWuLimit,
McPara_BkPlcMc_VeloAmpOutLimit,
McPara_BkPlcMc_FeedForward,

McPara_BkPlcMc_LagAmpTd,
McPara_BkPlcMc_LagAmpTdd,
McPara_BkPlcMc_LagAmpCfb_tA,
McPara_BkPlcMc_LagAmpCfb_kA,
McPara_BkPlcMc_LagAmpCfb_tV,
McPara_BkPlcMc_LagAmpCfb_kV,
McPara_BkPlcMc_LagCtrlType,
McPara_BkPlcMc_LagAmpCfb_tF,

106

Version: 1.8.3

TF5810

PLCopen Motion Control

McPara_BkPlcMc_LagAmpCfb_kF,
McPara_BkPlcMc_AccFeedForward,

McPara_BkPlcMc_Pctrl_kP:=1780,
McPara_BkPlcMc_Pctrl_Tn,
McPara_BkPlcMc_Pctrl_Tv,
McPara_BkPlcMc_Pctrl_Nf,
McPara_BkPlcMc_Pctrl_Preset,
McPara_BkPlcMc_Pctrl_WuLimit,
McPara_BkPlcMc_Pctrl_AlignAreas,

McPara_BkPlcMc_MonPositionRange:=1800,
McPara_BkPlcMc_MonTargetRange,
McPara_BkPlcMc_MonTargetFilter,
McPara_BkPlcMc_MonPositionLagFilter,
McPara_BkPlcMc_MonDynamicLagLimit,
McPara_BkPlcMc_MonPehEnable,
McPara_BkPlcMc_MonPehTimeout,
McPara_BkPlcMc_DigInputReversed,

McPara_PFW_EnableLimitPos:=1898,
McPara_PFW_EnableLimitNeg:=1899,

McPara_BkPlcMc_JogVeloFast:=1900,
McPara_BkPlcMc_JogVeloSlow,

McPara_BkPlcMc_CustomerData:=2000,

McPara_BkPlcMc_AutoId_EnaEoT:=3000,
McPara_BkPlcMc_AutoId_EnaOvl,
McPara_BkPlcMc_AutoId_EnaZadj,
McPara_BkPlcMc_AutoId_EnaAratio,
McPara_BkPlcMc_AutoId_EnaLinTab,
McPara_BkPlcMc_AutoId_EoT_N:=3100,
McPara_BkPlcMc_AutoId_EoT_P,
McPara_BkPlcMc_AutoId_EoI_N,
McPara_BkPlcMc_AutoId_EoI_P,
McPara_BkPlcMc_AutoId_EoTlim_N,
McPara_BkPlcMc_AutoId_EoTlim_P,
McPara_BkPlcMc_AutoId_DecFactor,
McPara_BkPlcMc_AutoId_EoVlim_N,
McPara_BkPlcMc_AutoId_EoVlim_P,
McPara_BkPlcMc_AutoId_LastIdent_N,
McPara_BkPlcMc_AutoId_LastIdent_P,
McPara_BkPlcMc_AutoId_TblCount:=3150,
McPara_BkPlcMc_AutoId_TblLowEnd,
McPara_BkPlcMc_AutoId_TblHighEnd,
McPara_BkPlcMc_AutoId_TblRamp,
McPara_BkPlcMc_AutoId_TblSettling,
McPara_BkPlcMc_AutoId_TblRecovery,
McPara_BkPlcMc_AutoId_TblMinCycle,
McPara_BkPlcMc_AutoId_LinTblAvailable,
McPara_BkPlcMc_AutoId_TblValveType,
McPara_BkPlcMc_AutoId_LinTab_1:=3200,
McPara_BkPlcMc_AutoId_LinTab_2:=3400,
(* ---------------------------------------------------------- *)
McRtData_BkPlcMc_ActualPosition:=10000,
McRtData_BkPlcMc_ActualAcceleration,
McRtData_BkPlcMc_PosError,
McRtData_BkPlcMc_DistanceToTarget,
McRtData_BkPlcMc_ActPressure,
McRtData_BkPlcMc_ActPressureA,
McRtData_BkPlcMc_ActPressureB,
McRtData_BkPlcMc_ActForce,
McRtData_BkPlcMc_ValvePressure,
McRtData_BkPlcMc_SupplyPressure,
McRtData_BkPlcMc_SetPosition,
McRtData_BkPlcMc_SetVelocity,
McRtData_BkPlcMc_SetAcceleration,
McRtData_BkPlcMc_SetPressure,
McRtData_BkPlcMc_SetOverride,
McRtData_BkPlcMc_LatchPosition,
McRtData_BkPlcMc_CtrlOutLag,
McRtData_BkPlcMc_CtrlOutClamping,
McRtData_BkPlcMc_CtrlOutOverlapComp,
McRtData_BkPlcMc_TargetPosition,
McRtData_BkPlcMc_NSDW:=11000,
McRtData_BkPlcMc_DCDW,
McRtData_BkPlcMc_ErrCode,
McRtData_BkPlcMc_FbState,

TF5810

Version: 1.8.3

107

PLCopen Motion Control

McRtData_BkPlcMc_CurStep,
McRtData_BkPlcMc_ParamsUnsave,
McRtData_BkPlcMc_RawPosition,
McRtData_BkPlcMc_ActPosCams,
McRtData_BkPlcMc_ReloadParams,
McRtData_BkPlcMc_EncoderMinPos,
McRtData_BkPlcMc_EncoderMaxPos,
McRtData_BkPlcMc_BufferedEntries,
McRtData_BkPlcMc_Pump_Switched:=12000,
McRtData_BkPlcMc_Pump_AreaSwitched,
McRtData_BkPlcMc_Pump_Angle:=12100,
McRtData_BkPlcMc_Pump_ModuloAngle,
McRtData_BkPlcMc_Pump_Speed,
McRtData_BkPlcMc_Pump_Torque,
McRtData_BkPlcMc_Motor_N_max,
McRtData_BkPlcMc_Active_Area_P,
McRtData_BkPlcMc_Active_Area_M,
McRtData_BkPlcMc_Active_Qmax_P,
McRtData_BkPlcMc_Active_Qmax_M,
McRtData_BkPlcMc_Active_Feed_P,
McRtData_BkPlcMc_Active_Feed_M,
McRtData_BkPlcMc_Active_N_max,
McRtData_BkPlcMc_Active_Vmax_P,
McRtData_BkPlcMc_Active_Vmax_M,
(* ---------------------------------------------------------- *)
(**)
McPara_BkPlcMc_
(**)
McPara_BkPlcMc_FileMarkComplete:=32767  (* Ax.Params.bLinTabAvailable AutoIdent: .. / AutoIdent: ..
*)229
);
END_TYPE

108

Version: 1.8.3

TF5810

Values

PLCopen Motion Control

TF5810

Version: 1.8.3

109

PLCopen Motion Control

Name
McPara_CommandedPosition:=1
McPara_SWLimitPos
McPara_SWLimitNeg
McPara_EnableLimitPos

McPara_EnableLimitNeg

McPara_EnablePosLagMonitoring
McPara_MaxPositionLag
McPara_MaxVelocitySystem

McPara_MaxVelocityAppl

McPara_ActualVelocity
McPara_CommandedVelocity
McPara_MaxAccelerationSystem

McPara_MaxAccelerationAppl

McPara_MaxDecelerationSystem

McPara_MaxDecelerationAppl

McPara_MaxJerk

McPara_BkPlcMc_ProfilType:=1000
McPara_BkPlcMc_EnvCycletime

McPara_BkPlcMc_AxName
McPara_BkPlcMc_TimeBased

McPara_BkPlcMc_JerkEnabled
McPara_BkPlcMc_LogLevel
McPara_BkPlcMc_CycleDivider
McPara_BkPlcMc_ParamFileName
McPara_BkPlcMc_EncoderType:=1100
McPara_BkPlcMc_EncoderHomingType

McPara_BkPlcMc_EncoderZeroShift

McPara_BkPlcMc_EncoderIncWeighting
McPara_BkPlcMc_EncoderIncInterpolation

McPara_BkPlcMc_EncoderRefIndexVelo

McPara_BkPlcMc_EncoderRefIndexPositive

McPara_BkPlcMc_EncoderRefSyncVelo

Description
The last commanded target position of the axis.
Software limit switch in positive direction.
Software limit switch in negative direction.
Enable for the software limit switch in positive
direction.
Enable for the software limit switch in negative
direction.
Enabling the lag error monitoring.
Threshold value for position lag monitoring.
The upper limit set by the system for the maximum
velocity that can be commanded by the application.
The maximum velocity that can be commanded by
the application.
The actual axis velocity.
The last commanded velocity of the axis.
The upper limit set by the system for the maximum
acceleration that can be commanded by the
application.
The maximum acceleration that can be commanded
by the application.
The upper limit set by the system for the maximum
deceleration that can be commanded by the
application.
The maximum deceleration that can be commanded
by the application.
The upper limit set by the system for the maximum
jerk that can be commanded by the application.
: Type of setpoint generation.
The cycle time of the task in which the core function
blocks (encoder, setpoint generator, etc.) of the axis
are called.
The text-based name of the axis.
The switching of the setpoint generation: Timebased
or Displacementbased.
The control word for jerk limitation.
Threshold value for message logging.
Reserved, not implemented.
Name of the parameter file.
Type of encoder evaluation.
Axes with incremental encoder: The default method
of homing.
Axes with absolute encoder system: The zero offset
shift of the encoder evaluation.
The increment weighting of the encoder evaluation.
The increment interpolation of the encoder
evaluation.
Axes with incremental encoder: The homing
searches for the index (cam) with this velocity.
Axes with incremental encoder: The homing
searches for the index (cam) in positive direction.
Axes with incremental encoder: The homing
searches for the homing signal with this velocity.

110

Version: 1.8.3

TF5810

Name
McPara_BkPlcMc_EncoderRefSyncPositive

McPara_BkPlcMc_EncoderDefaultHomePosition,

McPara_BkPlcMc_EncoderReversed
McPara_BkPlcMc_EncoderBaseDistance
McPara_BkPlcMc_EncoderModuloBase
McPara_BkPlcMc_EncoderEnableLatch

McPara_BkPlcMc_EncoderLatchedPos
McPara_BkPlcMc_EncoderRefShift

McPara_BkPlcMc_EncoderRefFlag
McPara_BkPlcMc_EncoderPotiRgToRl

McPara_BkPlcMc_EncoderOverrunMask
McPara_BkPlcMc_EncoderPositionMask

McPara_BkPlcMc_EncoderZeroSwap

McPara_BkPlcMc_EncoderNoUpload

McPara_BkPlcMc_EncoderModuloMode
McPara_BkPlcMc_ValveOverlapCompP:=1200

McPara_BkPlcMc_ValveBendPointVelo

McPara_BkPlcMc_ValveBendPointOutput

McPara_BkPlcMc_ValveResponseTime
McPara_BkPlcMc_ValveOverlapCompM

McPara_BkPlcMc_CylinderArreaA:=1280

McPara_BkPlcMc_CylinderArreaB

McPara_BkPlcMc_DriveType:=1300
McPara_BkPlcMc_AreaRatio
McPara_BkPlcMc_DriveReversed
McPara_BkPlcMc_DriveDefaultPowerOk

McPara_BkPlcMc_DriveAbsoluteOutput

McPara_BkPlcMc_DriveIncWeighting
McPara_BkPlcMc_DriveIncInterpolation
McPara_BkPlcMc_DriveNoUpload

McPara_BkPlcMc_DriveIsHybrid

PLCopen Motion Control

Description
Axes with incremental encoder: The homing
searches for the homing signal in positive direction.
Axes with incremental encoder: A default value for
homing.
Enable for inverted encoder evaluation.
Reserved for distance-coded encoders.
Reserved, not implemented.
Enable for the latch function of an encoder
hardware.
The latched position during homing.
Axes with incremental encoder: The zero offset shift
of the encoder evaluation.
Axis calibration state.
For potentiometer encoders: The ratio of total
potentiometer resistance to load resistance (input
resistance of the terminal).
A mask for detecting an encoder overflow.
A mask for isolating the valid bits within the mapped
variables.
Block-by-block shifting of the counting range of the
encoder evaluation.
A TRUE here prevents the automatic determination
of axis parameters by reading data from an encoder.
Reserved
Compensation of the valve overlap for the positive
direction.
Velocity for compensation of the characteristic curve
bend.
Valve output for compensation of the characteristic
curve bend.
Compensation of the valve response time.
Compensation of the valve overlap for the negative
direction.
The cylinder area of the cylinder side pushing in
positive direction.
The cylinder area of the cylinder side pushing in
negative direction.
Type of drive adjustment.
The direction-dependent velocity ratio.
Enable for inverted output adjustment.
Drive power is assumed to be available; no
hardware signal required.
Enable for absolute value formation during output
adjustment.
Weighting of the output adjustment.
Interpolation of the output adjustment.
A TRUE here prevents the automatic determination
of axis parameters by reading data from a drive.
This parameter is used to identify a servo-electric/
hydraulic hybrid axis.
Please note: A TRUE here triggers a recalculation of
drive parameters.

TF5810

Version: 1.8.3

111

PLCopen Motion Control

Name
McPara_BkPlcMc_HybridConcept

McPara_BkPlcMc_Pump_Cavities
McPara_BkPlcMc_Pump_EncType

McPara_BkPlcMc_Pump_N_max
McPara_BkPlcMc_Pump_N_min
McPara_BkPlcMc_Pump_P_max

McPara_BkPlcMc_Pump_P_min

McPara_BkPlcMc_Pump_Q_fast_P
McPara_BkPlcMc_Pump_Q_slow_P

McPara_BkPlcMc_Pump_Q_fast_M
McPara_BkPlcMc_Pump_Q_slow_M

McPara_BkPlcMc_Pump_Q_leak
McPara_BkPlcMc_Pump_Enc_Offset
McPara_BkPlcMc_Cylinder_A_addP

McPara_BkPlcMc_Cylinder_A_addM

McPara_BkPlcMc_PrsScaling_A
McPara_BkPlcMc_PrsScaling_B
McPara_BkPlcMc_PrsScaling_Sys
McPara_BkPlcMc_Motor_RampTime

McPara_BkPlcMc_Pump_Regenerative

McPara_BkPlcMc_Virtual_A_addP

McPara_BkPlcMc_Virtual_A_addM

McPara_BkPlcMc_Aside_PrsHiResADC

McPara_BkPlcMc_Bside_PrsHiResADC

McPara_BkPlcMc_System_PrsHiResADC

Description
The circuit concept used for the servo-electric/
hydraulic axis must be specified here.
Hybrid axis: Number of pump cavities
The encoder type of the pump drive is defined here.
Only a small selection of encoder types is available.

This is not the encoder on the cylinder.
[rpm] The maximum permissible pump speeds.
[rpm] The minimum permissible pump speeds.
[bar] The maximum permissible operating pressure
of the pump.
[bar] The minimum permissible operating pressure
of the pump.
[cm3/rev] The rotation-related flow rate of the pump
in rapid or force mode at the cylinder connection
acting in the positive direction.
[cm3/U] The rotation-related flow rate of the pump in
rapid or force mode at the cylinder connection acting
in the negative direction.
Reserved
Reserved
If, depending on the situation, an area effective for
oil demand is connected in the positive direction of
action, it must be identified here.
If, depending on the situation, an area effective for
oil demand is connected in the negative direction of
action, it must be identified here. This can also be an
oil demand required by an apparent area, which
actually bypasses the cylinder. In this case, the area
should be identified as "virtual".
The scaling pressures for the A-side, the B-side and
the system pressure detection are to be set here.

When switching between rapid and force mode, the
weighting factor for the velocity output and the
maximum attainable velocity are changed. A ramp
can be defined here, in order to avoid a
discontinuity.
This parameter indicates that the smaller cylinder
area is operated in oil exchange with the larger
cylinder area.
The active area that can be activated in positive
direction must be taken into account for the oil
demand, but it does not contribute to the force build-
up.
The active area that can be activated in negative
direction must be taken into account for the oil
demand, but it does not contribute to the force build-
up.
The pressure sensor on the area acting in positive
direction is read in with a 24-bit terminal.
The pressure sensor on the area acting in negative
direction is read in with a 24-bit terminal.
The pressure sensor at the pressurized hydraulic
reservoir is read with a 24-bit terminal.

112

Version: 1.8.3

TF5810

Name
McPara_BkPlcMc_StartRamp:=1400

McPara_BkPlcMc_obsolete_1
McPara_BkPlcMc_obsolete_2
McPara_BkPlcMc_StopRamp:=1500

McPara_BkPlcMc_EmergencyRamp

McPara_BkPlcMc_BrakeOn

McPara_BkPlcMc_BrakeOff

McPara_BkPlcMc_BrakeSafety

McPara_BkPlcMc_CreepSpeedP:=1600

McPara_BkPlcMc_CreepDistanceP

McPara_BkPlcMc_BrakeDistanceP

McPara_BkPlcMc_BrakeDeadTimeP
McPara_BkPlcMc_CreepSpeedM

McPara_BkPlcMc_CreepDistanceM

McPara_BkPlcMc_BrakeDistanceM

McPara_BkPlcMc_BrakeDeadTimeM
McPara_BkPlcMc_AsymetricalTargeting

McPara_BkPlcMc_LagAmp:=1700

McPara_BkPlcMc_LagAmpAdaptLimit
McPara_BkPlcMc_LagAmpAdaptFactor
McPara_BkPlcMc_ZeroCompensation
McPara_BkPlcMc_TargetClamping
McPara_BkPlcMc_ReposDistance
McPara_BkPlcMc_AutoBrakeDistance

McPara_BkPlcMc_EnableControlLoopOnFault
McPara_BkPlcMc_LagAmpDx

McPara_BkPlcMc_LagAmpTi

McPara_BkPlcMc_LagAmpWuLimit

McPara_BkPlcMc_LagAmpOutLimit
McPara_BkPlcMc_VeloAmp

PLCopen Motion Control

Description
Only for certain setpoint generators: The
acceleration ramp.
Reserved
Reserved
Only for certain setpoint generators: The
deceleration ramp.
In the event of an emergency stop: The time for
braking from maximum speed to standstill.
A delay between the signal for releasing a brake and
the active axis motion.
A delay between the active axis motion and the
signal for activating a brake.
A delay between the active axis motion in one
direction and active motion in the opposite direction.
The creep speed in positive direction. With
symmetric target approach: The creep velocity in
negative direction.
The creep distance in positive direction. With
symmetrical target approach: The creep distance in
negative direction.
The braking distance time in positive direction. For
symmetrical target approach: The braking distance
in negative direction.
The brake dead time in positive direction.
With asymmetric target approach: The creep velocity
in negative direction.
With asymmetric target approach: The creep
distance in negative direction.
For asymmetric target approach: The braking
distance in negative direction.
The brake dead time in negative direction.
Enable asymmetrical (direction-dependent) target
approach.
Gain factor for the proportional component in the
position controller.
Reserved
Reserved
Offset compensation of the output.
Default output value for the clamping function.
Threshold value for automatic repositioning.
Enable for automatic calculation of the braking
distance.
Enable for position control in the event of axis errors.
Threshold value for the integrating component of the
position controller.
Time constant for the integrating component of the
position controller.
Limitation (wind-up limit) for the integrating
component of the position controller.
Output limitation for the position controller.
Gain factor for the proportional component in the
velocity controller.

TF5810

Version: 1.8.3

113

PLCopen Motion Control

Name
McPara_BkPlcMc_VeloAmpDx

McPara_BkPlcMc_VeloAmpTi

McPara_BkPlcMc_VeloAmpWuLimit

McPara_BkPlcMc_VeloAmpOutLimit
McPara_BkPlcMc_FeedForward
McPara_BkPlcMc_LagAmpTd

McPara_BkPlcMc_LagAmpTdd

McPara_BkPlcMc_LagAmpCfb_tA

McPara_BkPlcMc_LagAmpCfb_kA

McPara_BkPlcMc_LagAmpCfb_tV

McPara_BkPlcMc_LagAmpCfb_kV

McPara_BkPlcMc_LagCtrlType
McPara_BkPlcMc_LagAmpCfb_tF
McPara_BkPlcMc_LagAmpCfb_kF
McPara_BkPlcMc_AccFeedForward
McPara_BkPlcMc_Pctrl_kP:=1780
McPara_BkPlcMc_Pctrl_Tn

McPara_BkPlcMc_Pctrl_Tv

McPara_BkPlcMc_Pctrl_Nf

McPara_BkPlcMc_Pctrl_Preset

McPara_BkPlcMc_Pctrl_WuLimit

McPara_BkPlcMc_Pctrl_AlignAreas

McPara_BkPlcMc_MonPositionRange:=1800
McPara_BkPlcMc_MonTargetRange
McPara_BkPlcMc_MonTargetFilter
McPara_BkPlcMc_MonPositionLagFilter
McPara_BkPlcMc_MonDynamicLagLimit
McPara_BkPlcMc_MonPehEnable

Description
Threshold value for the integrating component of the
velocity controller.
Time constant for the integrating component of the
velocity controller.
Limitation (wind up limit) for the integrating
component of the velocity controller.
Output limitation for the velocity controller.
Feed forward weighting of the axis.
A parameter of the extended position controller in
the MC_AxRtPosPiControllerEx_BkPlcMc [} 185]
function block: The gain of the D part.

A parameter of the extended position controller in
the MC_AxRtPosPiControllerEx_BkPlcMc [} 185]
function block: The attenuation of the D part.
A parameter of the extended position controller in
the MC_AxRtPosPiControllerEx_BkPlcMc function
block: The filter time of the actual acceleration
feedback.
A parameter of the extended position controller in
the MC_AxRtPosPiControllerEx_BkPlcMc function
block: The gain of the actual acceleration feedback.
A parameter of the extended position controller in
the MC_AxRtPosPiControllerEx_BkPlcMc function
block: The filter time of the actual velocity feedback.
A parameter of the extended position controller in
the MC_AxRtPosPiControllerEx_BkPlcMc function
block: The gain of the actual velocity feedback.
Type of lag controller
Force damping of the condition feedback in the lag
controller

Acceleration feed forward weighting
Proportional gain of a force or pressure controller.
Time constant in the I part of a force or pressure
controller.
Time constant in the D part of a force or pressure
controller.
Depth of a mean value filter in the D part of a force
or pressure controller.
Initial value of the I part of a force or pressure
controller.
Limitation of the I part of a force or pressure
controller.
Adaptation of the output of a force or pressure
controller to the direction-dependent active areas.
Tolerance for the position window.
Tolerance for the target window.
Filter time for the target window.
Filter time for position lag monitoring.
Tolerance for dynamic position lag monitoring.
Enable for monitoring of the ready message at the
target.

114

Version: 1.8.3

TF5810

Name
McPara_BkPlcMc_MonPehTimeout

McPara_BkPlcMc_DigInputReversed

McPara_PFW_EnableLimitPos:=1898
McPara_PFW_EnableLimitNeg:=1899
McPara_BkPlcMc_JogVeloFast:=1900
McPara_BkPlcMc_JogVeloSlow
McPara_BkPlcMc_CustomerData:=2000

McPara_BkPlcMc_AutoId_EnaEoT:=3000

McPara_BkPlcMc_AutoId_EnaOvl

McPara_BkPlcMc_AutoId_EnaZadj
McPara_BkPlcMc_AutoId_EnaAratio

McPara_BkPlcMc_AutoId_EnaLinTab

McPara_BkPlcMc_AutoId_EoT_N:=3100

McPara_BkPlcMc_AutoId_EoT_P

McPara_BkPlcMc_AutoId_EoI_N

McPara_BkPlcMc_AutoId_EoI_P

McPara_BkPlcMc_AutoId_EoTlim_N

McPara_BkPlcMc_AutoId_EoTlim_P

McPara_BkPlcMc_AutoId_DecFactor

McPara_BkPlcMc_AutoId_EoVlim_N

McPara_BkPlcMc_AutoId_EoVlim_P

McPara_BkPlcMc_AutoId_LastIdent_N

McPara_BkPlcMc_AutoId_LastIdent_P

McPara_BkPlcMc_AutoId_TblCount:=3150

McPara_BkPlcMc_AutoId_TblLowEnd

McPara_BkPlcMc_AutoId_TblHighEnd

PLCopen Motion Control

Description
Filter time for monitoring of the ready message at
the target.
Enable for inversion of the input signals of an axis
with digital position sensors.
Reserved
Reserved
A default value for a fast jog velocity.
A default value for a slow jog velocity.
A field with parameters that can be used freely by an
application. These parameters are stored and
loaded with the axis parameters.
Automatic identification: Determination of the hard
stops of the cylinder.
Automatic identification: Determination of the valve
overlap.
Automatic identification: Determination of the offset.
Automatic identification: Determination of the
direction-related velocity ratio.
Automatic identification: Determination of the
characteristic curve.
Automatic identification: Hard stop of the cylinder in
negative direction.
Automatic identification: Hard stop of the cylinder in
positive direction.
Automatic identification: Determined negative hard
stop of the cylinder in increments.
Automatic identification: Determined positive hard
stop of the cylinder in increments.
Automatic identification: Determined negative hard
stop of the cylinder.
Automatic identification: Determined positive hard
stop of the cylinder.
Automatic identification: Factor for deceleration
weighting.
Automatic identification: Velocity limitation of the
characteristic curves determination in negative
direction.
Automatic identification: Velocity limitation of the
characteristic curves determination in positive
direction.
Automatic identification: The output value of the last
successful measurement in negative direction.
Automatic identification: The output value of the last
successful measurement in positive direction.
Automatic identification: The number of table points
required. Since the zero point is counted but is only
present once, this parameter must always be an odd
number. Values between 3 and 99 are accepted. A
value of less than 11 is not recommended.
Automatic identification: Lower end of the intended
measuring distance.
Automatic identification: Upper end of the intended
measuring distance.

TF5810

Version: 1.8.3

115

PLCopen Motion Control

Name
McPara_BkPlcMc_AutoId_TblRamp

McPara_BkPlcMc_AutoId_TblSettling

McPara_BkPlcMc_AutoId_TblRecovery

McPara_BkPlcMc_AutoId_TblMinCycle

McPara_BkPlcMc_AutoId_LinTblAvailable

McPara_BkPlcMc_AutoId_TblValveType

McPara_BkPlcMc_AutoId_LinTab_1:=3200

McPara_BkPlcMc_AutoId_LinTab_2:=3400

McRtData_BkPlcMc_ActualPosition:=10000
McRtData_BkPlcMc_ActualAcceleration
McRtData_BkPlcMc_PosError
McRtData_BkPlcMc_DistanceToTarget
McRtData_BkPlcMc_ActPressure
McRtData_BkPlcMc_ActPressureA

McRtData_BkPlcMc_ActPressureB

McRtData_BkPlcMc_ActForce
McRtData_BkPlcMc_ValvePressure
McRtData_BkPlcMc_SupplyPressure
McRtData_BkPlcMc_SetPosition
McRtData_BkPlcMc_SetVelocity
McRtData_BkPlcMc_SetAcceleration
McRtData_BkPlcMc_SetPressure
McRtData_BkPlcMc_SetOverride
McRtData_BkPlcMc_LatchPosition

McRtData_BkPlcMc_CtrlOutLag
McRtData_BkPlcMc_CtrlOutClamping
McRtData_BkPlcMc_CtrlOutOverlapComp

McRtData_BkPlcMc_TargetPositio

McRtData_BkPlcMc_NSDW:=11000

McRtData_BkPlcMc_DCDW

Description
Automatic identification: Ramp for setting up the
measurement output. The specified time refers a
change from zero to full scale. Smaller output
changes are applied in a proportion of the time.
Automatic identification: Delay time between
establishment of the measuring output and the start
of the measurement.
Automatic characteristic curve identification: Waiting
time for a change of direction.
Automatic identification: Minimum measuring
distance.
This signal is set to TRUE at the end of a successful
characteristic curve measurement.
Automatic identification: The expected type of
characteristic curve.
Automatic identification: Points of the characteristic
curve, related velocity.
Automatic identification: Points of the characteristic
curve, related output.
The actual position.
The actual acceleration.
The lag error.
The remaining distance to the target.
The actual differential pressure at the valve.
The actual pressure in the A-chamber of the
cylinder.
The actual pressure in the B-chamber of the
cylinder.
The actual force.
The pressure drop at the valve.
The actual supply pressure value.
The current position setpoint.
The current velocity setpoint.
The current acceleration setpoint.
The setpoint for pressure or force regulators.
The current override value.
The (offset) reference position. This is the position at
which the actual position was finally set during
homing.
The current output of the position controller.
The current value of the terminal output.
The current output component of the overlap
compensation.
The last commanded target position of the axis.

This position is not changed by a Stop command.
The axis status word with the operating states.

There is no relationship with the status word of an
external device.
The control word of the axis with the enables (and
other parameters).

There is no relationship with the control word of an
external device.

116

Version: 1.8.3

TF5810

Name
McRtData_BkPlcMc_ErrCode
McRtData_BkPlcMc_FbState

McRtData_BkPlcMc_CurStep

McRtData_BkPlcMc_ParamsUnsave

McRtData_BkPlcMc_RawPosition

McRtData_BkPlcMc_ActPosCams

McRtData_BkPlcMc_ReloadParams

McRtData_BkPlcMc_EncoderMinPos
McRtData_BkPlcMc_EncoderMaxPos
McRtData_BkPlcMc_BufferedEntries

McRtData_BkPlcMc_Pump_Switched:=12000
McRtData_BkPlcMc_Pump_AreaSwitched
McRtData_BkPlcMc_Pump_Angle:=12100
McRtData_BkPlcMc_Pump_ModuloAngle

McRtData_BkPlcMc_Pump_Speed
McRtData_BkPlcMc_Pump_Torque

McRtData_BkPlcMc_Motor_N_max

McRtData_BkPlcMc_Active_Area_P

McRtData_BkPlcMc_Active_Area_M

McRtData_BkPlcMc_Active_Qmax_P

McRtData_BkPlcMc_Active_Qmax_M

McRtData_BkPlcMc_Active_Feed_P

McRtData_BkPlcMc_Active_Feed_M

McRtData_BkPlcMc_Active_N_max

McRtData_BkPlcMc_Active_Vmax_P

McRtData_BkPlcMc_Active_Vmax_M

McPara_BkPlcMc_FileMarkComplete

PLCopen Motion Control

Description
The current error code of the axis.
The current axis step (defined by PLCopen). See
also E_TcMCFbState.
The current (internal) axis step. See also
E_TcMcCurrentStep.
A TRUE here indicates that a parameter was
changed significantly, but the parameter file was not
yet written again. This signal cannot be issued by
the library, if the parameter was changed directly
(without the write function blocks).
The actual position, which was not manipulated
through a zero offset shift.
For axes with digital position sensors: The sensor
signals.
If parameters are changed by the runtime: A request
to the PlcMcManager to read out the parameters
again.
Reserved
Reserved
For axes with a command buffer: The number of
buffered commands.
With hybrid axes: The state of the pump switching.
With hybrid axes: The state of the area switching.
With hybrid axes: The pump actual angle.
With hybrid axes: The actual pump angle within the
current revolution.
With hybrid axes: The actual pump speed.
With hybrid axes: The actual torque of the pump
drive.
With hybrid axes: The speed limitation for the pump
drive.
With hybrid axes: The active area currently acting in
the positive direction.
With hybrid axes: The active area currently acting in
the negative direction.
With hybrid axes: The current delivery rate of the
pump acting in the positive direction.
With hybrid axes: The current delivery rate of the
pump acting in the negative direction.
With hybrid axes: The feed constant currently acting
in the positive direction.
With hybrid axes: The feed constant currently acting
in the negative direction.
With hybrid axes: The current speed limitation for
the pump.
With hybrid axes: The current maximum velocity in
the positive direction.
With hybrid axes: The current maximum velocity in
the negative direction.
In a parameter file: The logical end ID.

McPara_BkPlcMc_AsymmetricalTargeting: The enable for asymmetric targeting.

McPara_BkPlcMc_AutoID_EnaEoI_N: Automatic identification: Determined negative hard stop of the
cylinder in increments.

TF5810

Version: 1.8.3

117

PLCopen Motion Control

McPara_BkPlcMc_AutoID_EnaEoI_P: Automatic identification: Determined positive hard stop of the
cylinder in increments.

McPara_BkPlcMc_AutoID_MinCycle: Automatic identification: Minimum measuring distance.

McPara_BkPlcMc_Auto_BrakeDistance: The enable for the automatic calculation of the braking distance.

McPara_BkPlcMc_CycleDevider: Reserved, not implemented.

McPara_BkPlcMc_DigInputsReversed: Enable for inversion of the input signals of an axis with digital
position sensors.

McPara_BkPlcMc_EnableControlLoopOnFaults: The enable for position control in case of axis errors.

McPara_BkPlcMc_EncNoUpload: A TRUE here prevents the automatic determination of axis parameters
by reading data from an encoder.

McPara_BkPlcMc_EncoderLatchedPosition: The position latched during a homing.

McPara_BkPlcMc_obsolete_XYZ: Placeholder for parameters that are no longer supported. These
parameter codes must not be reused for new parameters. To ensure this, such numerical values are
assigned names of this form.

McPara_BkPlcMc_VelopWuLimit: Limitation (wind-up limit) for the integrating component of the velocity
controller.

McPara_PFW_Xyz: These parameters are reserved for a sector-specific solution.

McRtData_BkPlcMc_AxName: The textual name of the axis.

McRtData_BkPlcMc_FileMarkComplete: In a parameter file: The logical end identifier.

4.3.9

E_TcMcProfileType

Available from version 3.0

The constants listed here are used to identify the rules used to generate the control value for an axis.

Syntax
TYPE E_TcMcProfileType :
(
(*
The sequence below must not be changed!
New types have to be added at the end.
In case a type becomes obsolete it has to be replaced by a dummy
to ensure the numerical meaning of the other codes.
*)
(*
Die bestehende Reihenfolge darf nicht veraendert werden.
Neue Typen muessen am Ende eingefuegt werden.
Wenn ein Typ wegfallen sollte, muss er durch einen Dummy
ersetzt werden, um die numerische Zuordnung zu garantieren.
*)
iTcMc_ProfileConstAcc,
iTcMc_ProfileTimePosCtrl,
iTcMc_ProfileCosine,
iTcMc_ProfileCtrlBased,
iTcMc_ProfileTimeRamp,
iTcMc_ProfileJerkBased,
iTcMc_ProfileBufferedJerk,
iTcMc_ProfileSwitchedVelo,
iTcMc_Profile_TestOnly:=100
);
END_TYPE

118

Version: 1.8.3

TF5810

Values

Name
iTcMc_ProfileConstAcc

iTcMc_ProfileTimePosCtrl
iTcMc_ProfileCosine
iTcMc_ProfileCtrlBased

iTcMc_ProfileTimeRamp

iTcMc_ProfileJerkBased

iTcMc_ProfileBufferedJerk
iTcMc_ProfileSwitchedVelo
iTcMc_Profile_TestOnly

PLCopen Motion Control

Description
Only present for compatibility reasons; has been replaced by
iTcMc_ProfileCtrlBased.
Only present for compatibility reasons; no longer supported.
Only present for compatibility reasons; no longer supported.
The control value for the drive is assembled from sections of constant
acceleration and deceleration. Time (acceleration, change of velocity,
stop) and distance (positioning) function as controlling values.

This generator type can optionally operate in purely timer-controlled
mode with continuously closed position controller.
The control value for the drive is generated with time-controlled ramps
for accelerations and decelerations. The controlling parameters are time
(acceleration, velocity change, stop) and path (braking, stopping).

This generator type is intended for axes, which only have digital cams
instead of an encoder.
The control value for the drive is assembled from sections of constant
acceleration and deceleration. The deceleration is reduced with limited
jerk towards the target. Optionally, the acceleration can be increased
with limited jerk. Time (acceleration, change of velocity, stop) and
distance (positioning) function as controlling values.

Some functions are not supported by this generator type, or not fully.

This generator type can optionally operate in purely timer-controlled
mode with continuously closed position controller.
Reserved
Reserved for sector-specific packet.
This type is only intended for internal testing of function block prototypes,
which have not yet been released. It cannot be set via the
PlcMcManager.

4.3.10

E_TcMcPressureReadingMode

Available from version 3.0

The constants in this list are transferred to function blocks for logging actual force or pressure values [} 20].
They determine which actual value should be updated in the ST_TcHydAxRtData [} 141] structure with the
result of the evaluation.

Syntax
TYPE E_TcMcPressureReadingMode :
(
    iTcHydPressureReadingDefault,
    iTcHydPressureReadingActPressure,
    iTcHydPressureReadingActPressureA,
    iTcHydPressureReadingActPressureB,
    iTcHydPressureReadingActForce,
    iTcHydPressureReadingSupplyPressure,
    iTcHydPressureReadingValvePressure
);
END_TYPE

TF5810

Version: 1.8.3

119

PLCopen Motion Control

Values

Name
iTcHydPressureReadingDefault

iTcHydPressureReadingActPressure

iTcHydPressureReadingActPressureA
iTcHydPressureReadingActPressureB
iTcHydPressureReadingActForce

iTcHydPressureReadingSupplyPressure
iTcHydPressureReadingValvePressure

4.3.11

E_TcMcValveType

Description
The target variable depends on the function block being
used.
The target variable is fActPressure. Some function blocks
automatically update fActPressureA and fActPressureB.
The target variable is fActPressureA.
The target variable is fActPressureB.
The target variable is fActForce. Some function blocks
automatically update fActPressure,fActPressureA and
fActPressureB.
The target variable is fSupplyPressure.
The target variable is fValvePressure.

The constants in this list are used to mark rules for automatically identifying characteristic curves of an axis.

Syntax
TYPE E_TcMcValveType :
(
(*
The sequence below must not be changed!
New types have to be added at the end.
In case a type becomes obsolete it has to be replaced by a dummy
to ensure the numerical meaning of the other codes.
*)
(*
Die bestehende Reihenfolge darf nicht veraendert werden.
Neue Typen muessen am Ende eingefuegt werden.
Wenn ein Typ wegfallen sollte, muss er durch einen Dummy
ersetzt werden, um die numerische Zuordnung zu garantieren.
*)
iTcMc_ValveTypeDefault,
iTcMc_ValveTypeAbrupt,
iTcMc_ValveTypeDecomp,
iTcMc_ValveTypeLinearP,
iTcMc_ValveTypeLinearM,
iTcMc_ValveTypeCopyToP,
iTcMc_ValveTypeCopyToM
);
END_TYPE

120

Version: 1.8.3

TF5810

Values

Name
iTcMc_ValveTypeDefault
iTcMc_ValveTypeAbrupt

iTcMc_ValveTypeDecomp

iTcMc_ValveTypeLinearP

iTcMc_ValveTypeLinearM

iTcMc_ValveTypeCopyToP

iTcMc_ValveTypeCopyToM

PLCopen Motion Control

Description
Standard method: Measurement in both directions of movement.
This setting is provided on valves with an abrupt transition from the
coverage area. This is only the case with very few valve variants, and
without this setting it manifests itself through a very hard behavior,
especially at the beginning of the automatic identification.

Notice This setting should only be made in coordination with
Hydraulic Support.
This setting is adapted to valves with pressure relief in the coverage area
(h symbol).
With this setting, the identification is performed only in the negative
direction. For the positive direction a linear characteristic curve is
assumed, the endpoint of which is calculated from the maximum velocity
in the negative direction using the set velocity ratio.

Notice The velocity ratio is not determined automatically.
With this setting, the identification is carried out only in the positive
direction. For the negative direction a linear characteristic curve is
assumed, the endpoint of which is calculated from the maximum velocity
in the positive direction using the set velocity ratio.

Notice The velocity ratio is not determined automatically.
With this setting, the identification is performed only in the negative
direction. For the positive direction, the measuring points are calculated
from the measuring points of the negative direction using the set velocity
ratio.

Notice The velocity ratio is not determined automatically.
With this setting, the identification is carried out only in the positive
direction. For the negative direction, the measuring points are calculated
from the measuring points of the positive direction using the set velocity
ratio.

Notice The velocity ratio is not determined automatically.

4.3.12

MC_BufferMode_BkPlcMc

Available from version 3.0

The constants in this list are used for controlling blending according to PLC Open.

Syntax
TYPE MC_BufferMode_BkPlcMc :
(
Aborting_BkPlcMc := 0,
Buffered_BkPlcMc,
BlendingLow_BkPlcMc,
BlendingPrevious_BkPlcMc,
BlendingNext_BkPlcMc,
BlendingHigh_BkPlcMc
);
END_TYPE

TF5810

Version: 1.8.3

121

PLCopen Motion Control

Values

Name
Aborting_BkPlcMc

Buffered_BkPlcMc

BlendingLow_BkPlcMc

BlendingPrevious_BkPlcMc

BlendingNext_BkPlcMc

BlendingHigh_BkPlcMc

Description
The default case: The new command becomes active immediately and
cancels any other command that may already be active. The function
block monitoring the aborted command will respond with
CommandAborted.
For axes with command buffer: This command is started automatically
once all previous commands have been fully processed.
For axes with command buffer: This command is connected to the
previous command without intermediate stop. If possible, the transition
point is passed with the lower velocity of the commands involved.
For axes with command buffer: This command is connected to the
previous command without intermediate stop. If possible, the transition
point is passed with the commanded velocity of the previous command.
For axes with command buffer: This command is connected to the
previous command without intermediate stop. If possible, the transition
point is passed with the commanded velocity of the new command.
For axes with command buffer: This command is connected to the
previous command without intermediate stop. If possible, the transition
point is passed with the higher velocity of the commands involved.

4.3.13

MC_CAM_ID_BkPlcMc

Available from version 3.0

(internal use only).

Syntax
TYPE MC_CAM_ID_BkPlcMc:
STRUCT
    stCamRef:       MC_CAM_REF_BkPlcMc;
    bValidated:     BOOL:=FALSE;
    bPeriodic:      BOOL:=FALSE;
    bMasterAbs:     BOOL:=FALSE;
    bSlaveAbs:      BOOL:=FALSE;
    bIsChanged:     BOOL:=TRUE;
END_STRUCT
END_TYPE

Values

Name
stCamRef

bValidated

bPeriodic
bMasterAbs

bSlaveAbs

bIsChanged

Description

A copy of the MC_CAM_REF_BkPlcMc [} 123] structure.
Here this structure is identified as valid, if it was initialized by a function block of type
MC_CamTableSelect_BkPlcMc [} 53].
Reserved
Specifies whether the data of the master column are absolute or refer to the master position
at the time of the coupling.
Specifies whether the data of the slave column are absolute or refer to the slave position at
the time of the coupling.
Reserved

122

Version: 1.8.3

TF5810

PLCopen Motion Control

4.3.14

MC_CAM_REF_BkPlcMc

Available from version 3.0

(internal use only).

Syntax
TYPE MC_CAM_REF_BkPlcMc:
STRUCT
    pTable:        POINTER TO LREAL:=0;
    nFirstIdx:     UDINT:=1;
    nLastIdx:      UDINT:=1;
    bEquiDistant:  BOOL:=FALSE;
END_STRUCT
END_TYPE

Values

Name
pTable
nFirstIdx
nLastIdx
bEquiDistant

Description
The address of the curve table.
The index of the first table row.
The index of the last table row.
Reserved

4.3.15

MC_CAMSWITCH_REF_BkPlcMc

Available from version 3.0

A variable of this type is transferred to an MC_DigitalCamSwitch_BkPlcMc [} 54] function block.

Syntax
TYPE CAMSWITCH_REF_BkPlcMc:
STRUCT
    Switch:     ARRAY [ciBkPlcMc_CamSwitchRef_MinIdx..ciBkPlcMc_CamSwitchRef_MaxIdx] OF MC_CAMSWITCH
_REFTYPE_BkPlcMc;
END_STRUCT
END_TYPE

TYPE MC_CAMSWITCH_REFTYPE_BkPlcMc:
STRUCT
    TrackNumber:      INT;
    FirstOnPosition:  LREAL;
    LastOnPosition:   LREAL;
    AxisDirection:    INT;
    CamSwitchMode:    INT;
    Duration:         LREAL;
    (* private members, do not touch *)
    nCurrentState:    SINT:=0;
    bTriggered:       BOOL:=FALSE;
    fTimer:           LREAL;
    (**)
END_STRUCT
END_TYPE

TF5810

Version: 1.8.3

123

PLCopen Motion Control

Parameter

Name
TrackNumber

Type
INT

FirstOnPosition

LREAL

LastOnPosition
AxisDirection

LREAL
INT

CamSwitchMode

INT

Duration
nCurrentState
bTriggered
fTimer

LREAL
SINT
BOOL
LREAL

Description
This is an index in an ARRAY
[ciBkPlcMc_TrackRef_MinIdx..ciBkPlcMc_TrackRef_MaxIdx] OF
MC_TRACK_REF_BkPlcMc [} 126], which is transferred to a function block of
type MC_DigitalCamSwitch_BkPlcMc [} 54].
[mm] The start of the cam track. For time-controlled cams, this is the trigger
position.
[mm] The end of the cam track. Has no effect for time-controlled cams.
Specifies in which direction of movement the cam becomes active: 0 =
both directions, 1 = positive direction, 2 = negative direction.
The operating mode of the cam: For displacement-controlled cams enter 0,
for time-controlled cams enter 1.
[s] For time-controlled cams enter the switch-on time in seconds.
These elements are runtime variables and must not be influenced or used
by the application.

4.3.16

MC_Direction_BkPlcMc

Available from version 3.0

The constants listed here are used to identify the direction in which axes are moving.

Syntax
TYPE MC_Direction_BkPlcMc:
(
MC_Positive_Direction_BkPlcMc := 1,
MC_Shortest_Way_BkPlcMc,
MC_Negative_Direction_BkPlcMc,
MC_Current_Direction_BkPlcMc,
MC_SwitchPositive_Direction_BkPlcMc,
MC_SwitchNegative_Direction_BkPlcMc
);
END_TYPE

Values

Name
MC_Positive_Direction_BkPlcMc

MC_Shortest_Way_BkPlcMc

MC_Negative_Direction_BkPlcMc

MC_Current_Direction_BkPlcMc

MC_SwitchPositive_Direction_BkPlcMc
MC_SwitchNegative_Direction_BkPlcMc

Description
The movement is in the direction of rising values of
position.
The direction of movement is selected so that the
distance covered is as short as possible.
The movement is in the direction of falling values of
position.
The movement is in the same direction as the most
recently executed movement.
not supported
not supported

124

Version: 1.8.3

TF5810

PLCopen Motion Control

4.3.17

MC_HomingMode_BkPlcMc

Available from version 3.0

The constants in this list are used for identifying the modes during axis homing.

Syntax
TYPE MC_HomingMode_BkPlcMc:
(
    MC_DefaultHomingMode_BkPlcMc,
    MC_AbsSwitch_BkPlcMc,
    MC_LimitSwitch_BkPlcMc,
    MC_RefPulse_BkPlcMc,
    MC_Direct_BkPlcMc,
    MC_Absolute_BkPlcMc,
    MC_Block_BkPlcMc,
    MC_FlyingSwitch_BkPlcMc,
    MC_FlyingRefPulse_BkPlcMc
);
END_TYPE

Values

Name
MC_DefaultHomingMode_BkPlcMc

MC_AbsSwitch_BkPlcMc
MC_LimitSwitch_BkPlcMc
MC_RefPulse_BkPlcMc
MC_Direct_BkPlcMc
MC_Absolute_BkPlcMc
MC_Block_BkPlcMc
MC_FlyingSwitch_BkPlcMc
MC_FlyingRefPulse_BkPlcMc

Description
The referencing method specified in the axis parameters is
used.
The method iTcMc_HomingOnIndex is used.
not supported
The method iTcMc_HomingOnSync is used.
The method iTcMc_HomingOnExec is used.
not supported
The method iTcMc_HomingOnBlock is used.
not supported
not supported

4.3.18

MC_StartMode_BkPlcMc

Available from version 3.0

The constants in this list are used for identifying the modes during axis startups.

Syntax
TYPE MC_StartMode_BkPlcMc:
(
    MC_StartMode_Absolute:=1,
    MC_StartMode_Relative,
    MC_StartMode_RampIn
);
END_TYPE

TF5810

Version: 1.8.3

125

PLCopen Motion Control

Values

Name
MC_StartMode_Absolute

MC_StartMode_Relative

MC_StartMode_RampIn

Description
The set slave position determined by the MC_CamIn_BkPlcMc function
block is regarded as absolute value.
The set slave position determined by MC_CamIn_BkPlcMc function blocks
is regarded as distance from the location of the coupling.
Not supported

4.3.19

MC_TRACK_REF_BkPlcMc

Available from version 3.0

Syntax
TYPE TRACK_REF_BkPlcMc:
STRUCT
    Track:          ARRAY [ciBkPlcMc_TrackREF_MinIdx..ciBkPlcMc_TrackREF_MaxIdx] OF MC_TRACK_REFTYPE
_BkPlcMc;
END_STRUCT
END_TYPE

TYPE MC_TRACK_REFTYPE_BkPlcMc:
STRUCT
    OnCompensation: LREAL;
    OffCompensation:LREAL;
    Hysteresis:     LREAL;
END_STRUCT
END_TYPE

Parameter

Name
OnCompensation
OffCompensation
Hysteresis

Description
The switch-on dead time to be compensated in seconds.
The switch-off dead time to be compensated in seconds.
The axis must have moved away from the switching point by this distance before
reaching of the switching point is evaluated again.

If a positive value is specified as dead time compensation, signaling is delayed. A
negative value leads to leading signaling.

The time cannot be adhered to precisely, if the controlling parameter changes
with a fluctuating rate. If this controlling parameter is an actual axis position,
the actual axis velocity must be constant.

4.3.20

OUTPUT_REF_BkPlcMc

Available from version 3.0

A structure of this type is transferred to function blocks of types MC_ReadDigitalOutput_BkPlcMc() [} 34],
MC_WriteDigitalOutput_BkPlcMc() [} 47] and MC_DigitalCamSwitch_BkPlcMc() [} 54].

Syntax
TYPE OUTPUT_REF_BkPlcMc:
STRUCT
    OutputBits: UDINT:=0;
END_STRUCT
END_TYPE

126

Version: 1.8.3

TF5810

PLCopen Motion Control

Parameter

Name
OutputBits

Type
UDINT

Description
The outputs addressed via this structure.

4.3.21

ST_FunctionGeneratorFD_BkPlcMc

Available from version 3.0.31

This structure consolidates parameter for the definition of the output signals of a function generator. A
structure of this type is transferred to MC_FunctionGeneratorFD_BkPlcMc [} 226]() function blocks.

Syntax
TYPE ST_FunctionGeneratorFD_BkPlcMc :
STRUCT
     Sin_Amplitude:    LREAL:=0.0;
     Sin_Phase:        LREAL:=0.0;
     Sin_Offset:       LREAL:=0.0;

     Cos_Amplitude:    LREAL:=0.0;
     Cos_Phase:        LREAL:=0.0;
     Cos_Offset:       LREAL:=0.0;

     Rect_Amplitude:   LREAL:=0.0;
     Rect_Phase:       LREAL:=0.0;
     Rect_Ratio:       LREAL:=0.5;
     Rect_Offset:      LREAL:=0.0;

     Saw_Amplitude:    LREAL:=0.0;
     Saw_Phase:        LREAL:=0.0;
     Saw_Ratio:        LREAL:=0.5;
     Saw_Offset:       LREAL:=0.0;

END_STRUCT
END_TYPE

Parameter

Name
Sin_Amplitude

Cos_Amplitude

Rect_Amplitude

Saw_Amplitude
Sin_Phase

Cos_Phase

Rect_Phase

Saw_Phase
Sin_Offset

Cos_Offset

Rect_Offset

Saw_Offset
Rect_Ratio

Saw_Ratio

Type
LREAL

Description
The peak value of the signals.

LREAL

The phase shift of the signals.

LREAL

The zero offset of the signals.

LREAL

The duty factor of the square or sawtooth signal.

TF5810

Version: 1.8.3

127

PLCopen Motion Control

4.3.22

ST_FunctionGeneratorTB_BkPlcMc

Available from version 3.0.31

This structure consolidates parameters for the time base of one or several function generators. A structure of
this type is transferred to MC_FunctionGeneratorTB_BkPlcMc [} 228](), MC_FunctionGeneratorFD_BkPlcMc
[} 127]() and MC_FunctionGeneratorSetFrq_BkPlcMc [} 227]() function blocks.

Syntax
TYPE ST_FunctionGeneratorTB_BkPlcMc :
STRUCT
     Frequency:        LREAL:=0.000001;
     Freeze:           BOOL:=FALSE;

     CycleCount:       DINT:=0;
     CurrentTime:      LREAL:=0.0;
     CurrentRatio:     LREAL:=0.0;
END_STRUCT
END_TYPE

Parameter

Name
Frequency

Type
LREAL

Description
The operating frequency of the time base generated by an
MC_FunctionGeneratorTB_BkPlcMc [} 228]() function block in Hertz.

Freeze

BOOL

CycleCount
CurrentTime
CurrentRatio

DINT
LREAL
LREAL

If this variable is set to TRUE, a MC_FunctionGeneratorTB_BkPlcMc [} 228]()
function block will not evaluate the structure.
The number of fully generated signal sequences.
The time elapsed since the currently created signal sequence.
The normalized progress since the start of the currently generated signal
sequence.

4.3.23

ST_TcMcAutoIdent

Available from version 3.0.4

In this structure the parameters for an MC_AxUtiAutoIdent_BkPlcMc function block are stored. It contains
further information about the purpose of the individual elements.

Syntax
TYPE ST_TcMcAutoIdent :
(* last modification: 08.11.2019 *)
STRUCT
     EndOfTravel_Negativ:        LREAL:=0.0;
     EndOfTravel_Positiv:        LREAL:=0.0;
     EndOfTravel_NegativLimit:   LREAL:=0.0;
     EndOfTravel_PositivLimit:   LREAL:=0.0;
     DecelerationFactor:         LREAL:=1.0;
     EndOfVelocity_NegativLimit: LREAL:=0.0;
     EndOfVelocity_PositivLimit: LREAL:=0.0;
     EndOfTravel_LastIdent_P:    LREAL:=0.0;
     EndOfTravel_LastIdent_M:    LREAL:=0.0;
     ValveCharacteristicLowEnd:  LREAL:=0.0;
     ValveCharacteristicHighEnd: LREAL:=0.0;
     ValveCharacteristicRamp:    LREAL:=0.0;
     ValveCharacteristicSettling:LREAL:=0.0; (* starting with V3.0.32 *)
     ValveCharacteristicRecovery:LREAL:=0.0;
     ValveCharacteristicMinCycle:LREAL:=0.0;

     Valve_LinLimitP: LREAL:=0.0;   (* starting with V3.0.46 *)

128

Version: 1.8.3

TF5810

PLCopen Motion Control

     Valve_LinLimitM: LREAL:=0.0;

     ValveCharacteristicTable:   ARRAY[1..100,1..2] OF LREAL;

     EndOfIncrements_Negativ:    DINT:=0;
     EndOfIncrements_Positiv:    DINT:=0;

     ValveCharacteristicType:    INT:=0; (* starting with V3.0.33 *)
     ValveCharacteristicTblCount:INT:=0;

     EnableEndOfTravel:          BOOL:=FALSE;
     EnableOverlap:              BOOL:=FALSE;
     EnableZeroAdjust:           BOOL:=FALSE;
     EnableArreaRatio:           BOOL:=FALSE;
     EndOfTravel_PositivDone:    BOOL:=FALSE;
     EndOfTravel_NegativDone:    BOOL:=FALSE;
     EnableValveCharacteristic:  BOOL:=FALSE;
     EnableNoUturn: BOOL:=FALSE;
END_STRUCT
END_TYPE

Description

Parameter

Name
EndOfTravel_Negativ
EndOfTravel_Positiv
EndOfTravel_NegativLimit
EndOfTravel_PositivLimit
DecelerationFactor
EndOfVelocity_NegativLimit
EndOfVelocity_PositivLimit
EndOfTravel_LastIdent_P
EndOfTravel_LastIdent_M
ValveCharacteristicLowEnd
ValveCharacteristicHighEnd
ValveCharacteristicRamp
ValveCharacteristicSettling
ValveCharacteristicRecovery
ValveCharacteristicMinCycle
Valve_LinLimitP
Valve_LinLimitM
ValveCharacteristicTable
EndOfIncrements_Negativ
EndOfIncrements_Positiv
ValveCharacteristicType
ValveCharacteristicTblCount
EnableEndOfTravel
EnableOverlap
EnableZeroAdjust
EnableArreaRatio
EndOfTravel_PositivDone
EndOfTravel_NegativDone
EnableValveCharacteristic
EnableNoUturn

Type
LREAL
LREAL
LREAL
LREAL
LREAL
LREAL
LREAL
LREAL
LREAL
LREAL
LREAL
LREAL
LREAL
LREAL
LREAL
LREAL
LREAL
ARRAY
DINT
DINT
INT
INT
BOOL
BOOL
BOOL
BOOL
BOOL
BOOL
BOOL
BOOL

TF5810

Version: 1.8.3

129

PLCopen Motion Control

4.3.24

ST_TcHydAxParam

Available from version 3.0

This structure contains all axis parameters. Also see about this: Suitable procedures for axis commissioning
[} 380].

The order of the parameters is not guaranteed.

Syntax
TYPE ST_TcHydAxParam :
(* last modification: 08.07.2020 *)
STRUCT
    (* ==================================================================
    this section isn't saved / dieser Bereich wird nicht gesichert
    ================================================================== *)
    sParamFileName: STRING(255) := 'DefAxParmFile.dat';
    (* ==================================================================
    from this point all parameters are saved /
von hier an werden alle Parameter gesichert
    ================================================================== *)
    fAcc:                         LREAL := 2000.0;
    fAreaRatio:                   LREAL := 1.0;
    fBrakeDeadTimeM:              LREAL := 0.0;
    fBrakeDeadTimeP:              LREAL := 0.0;
    fBrakeDistanceM:              LREAL := 0.1;
    fBrakeDistanceP:              LREAL := 0.1;
    fBrakeOffDelay:               LREAL := 0.0;
    fBrakeOnDelay:                LREAL := 0.0;
    fBrakeSafetyDelay:            LREAL := 0.0;
    fCreepDistanceM:              LREAL := 1.0;
    fCreepDistanceP:              LREAL := 1.0;
    fCreepSpeedM:                 LREAL := 80.0;
    fCreepSpeedP:                 LREAL := 80.0;
    fCustomerData:                ARRAY [1..iTcHydfCustDataMaxIdx] OF LREAL;
    fCycletime:                   LREAL := 0.010;
    fCylinder_ArreaA:             LREAL := 1.0;
    fCylinder_ArreaB:             LREAL := 1.0;
    fCylinder_Mass:               LREAL := 1.0;
    fCylinder_Stroke:             LREAL := 1.0;
    fDec:                         LREAL := 2000.0;
    fDrive_IncInterpolation:      LREAL := 1.0;
    fDrive_IncWeighting:          LREAL := 0.001;
    fEmergencyRamp:               LREAL := 0.1;
    fEnc_BaseDistance:            LREAL := 0.001;
    fEnc_DefaultHomePosition:     LREAL := 0.0;
    fEnc_IncInterpolation:        LREAL := 1.0;
    fEnc_IncWeighting:            LREAL := 0.001;
    fEnc_ModuloBase:              LREAL := 0.001;
    fEnc_PotiRgToRl:              LREAL := 0.0;
    fEnc_RefIndexVelo:            LREAL := 0.1;
    fEnc_RefSyncVelo:             LREAL := 0.1;
    fEnc_ZeroShift:               LREAL := 0.0;
    fJogVeloFast:                 LREAL := 100.0;
    fJogVeloSlow:                 LREAL := 25.0;
    fFeedForward:                 LREAL := 1.0;
    fAccFeedForward:              LREAL := 0.0;
    fLagAmp:                      LREAL := 0.05;
    fLagAmpDp:                    LREAL := 0.0;
    fLagAmpDx:                    LREAL := 0.0;
    fLagAmpTi:                    LREAL := 0.0;
    fLagAmpOutL:                  LREAL := 0.0;
    fLagAmpWuL:                   LREAL := 0.0;
    fLagAmpTd:                    LREAL := 0.0;
    fLagAmpTdd:                   LREAL := 0.0;
    fLagAmpCfb_kV:                LREAL := 0.0;
    fLagAmpCfb_tV:                LREAL := 0.0;
    fLagAmpCfb_kA:                LREAL := 0.0;
    fLagAmpCfb_tA:                LREAL := 0.0;
    fLagAmpCfb_kF:                LREAL := 0.0;

130

Version: 1.8.3

TF5810

PLCopen Motion Control

    fLagAmpCfb_tF:                LREAL := 0.0;
    fMaxAcc:                      LREAL := 500.0;
    fMaxDec:                      LREAL := 500.0;
    fMaxDynamicLag:               LREAL := 0.0;
    fMaxJerk:                     LREAL := 1000.0;
    fMaxLag:                      LREAL := 0.0;
    fMaxLagFilter:                LREAL := 0.0;
    fMaxVelo:                     LREAL := 500.0;
    fMonPositionRange:            LREAL := 1.0;
    fMonTargetFilter:             LREAL := 1.0;
    fMonTargetRange:              LREAL := 1.0;
    fPEH_Timeout:                 LREAL := 0.0;
    fRefVelo:                     LREAL := 500.0;
    fReposDistance:               LREAL := 0.0;
    fSoftEndMax:                  LREAL := 10000.0;
    fSoftEndMin:                  LREAL := 0.0;
    fStartAccDistance:            LREAL := 1.0;
    fStartRamp:                   LREAL := 1.0;
    fStopRamp:                    LREAL := 1.0;
    fTargetClamping:              LREAL := 0.0;
    fVeloAmp:                     LREAL := 0.0;
    fVeloAmpDx:                   LREAL := 0.0;
    fVeloAmpTi:                   LREAL := 0.0;
    fVeloAmpOutL:                 LREAL := 0.0;
    fVeloAmpWuL:                  LREAL := 0.0;
    fValve_BendPointOutput:       LREAL := 0.0;
    fValve_BendPointVelo:         LREAL := 0.0;
    fValve_OverlapCompM:          LREAL := 0.0;
    fValve_OverlapCompP:          LREAL := 0.0;
    fValve_ResponseTime:          LREAL := 0.0;
    fZeroCompensation:            LREAL := 0.0;

    nEnc_OverrunMask:             DWORD := 0;
    nEnc_PositionMask:            DWORD := 0;
    nEnc_ZeroSwap:                DINT := 0;
    nDigInReversed:               DINT := 0;

    nCycleDivider:                INT := 1;
    nDrive_Type:                  E_TcMcDriveType:=iTcMc_Drive_Customized;
    nEnc_HomingType:              E_TcMcHomingType:=iTcMc_HomingOnBlock;
   nEnc_Type:                    E_TcMcEncoderType:=iTcMc_EncoderSim;

    nJerkEnabled:                 WORD := 16#0101;
    nProfileType:                 E_TcMcProfileType:=iTcMc_ProfileCtrlBased;
    nControllerType:              WORD := 16#0101;
    nOverlapDefMode:              WORD := 0;

    bAsymetricalTargeting:        BOOL := FALSE;
    bDrive_AbsoluteOutput:        BOOL := FALSE;
    bDrive_DefaultPowerOk:        BOOL := FALSE;
    bDrive_Reversed:              BOOL := FALSE;
    bEnableAutoBrakeDistance:     BOOL := FALSE;
    bEnableControlLoopOnFault:    BOOL := FALSE;
    bEnc_RefIndexPositive:        BOOL := FALSE;
    bEnc_RefSyncPositive:         BOOL := FALSE;

    bEnc_Reversed:                BOOL := FALSE;
    bMaxLagEna:                   BOOL := FALSE;
    bPEH_Enable:                  BOOL := FALSE;
    bPosCtrlAccEna:               BOOL := FALSE;
    bSoftEndMaxEna:               BOOL := FALSE;
    bSoftEndMinEna:               BOOL := FALSE;
    bTimeBased:                   BOOL := FALSE;
    bLinTabAvailable:             BOOL := FALSE;

    bEnc_NoUpLoad:                BOOL := FALSE;
    bDrive_NoUpLoad:              BOOL := FALSE;
    bDriveIsHybrid:               BOOL := FALSE;
    bAlignedStart:                BOOL := FALSE;
    bEncModuloMode:               BOOL := FALSE;

    (*-----------------------------------------------------------------*)

    stHybrid:                     ST_TcHybridAxParam;
    stPctrl:                      ST_TcPctrlParam;
END_STRUCT
END_TYPE

TF5810

Version: 1.8.3

131

PLCopen Motion Control

Parameter

132

Version: 1.8.3

TF5810

Name
sParamFileName

fAcc

fAreaRatio

Type
STRING

LREAL

LREAL

fBrakeDeadTimeM

LREAL

fBrakeDeadTimeP

LREAL

fBrakeDistanceM

LREAL

fBrakeDistanceP

LREAL

fBrakeOffDelay

LREAL

fBrakeOnDelay

LREAL

PLCopen Motion Control

Description
This file name is used for storing
the axis parameter as a DAT file.
[mm/s2] The absolute acceleration
limitation of the axis.
[1] This parameter can be used to
compensate the directional
dependence of the velocity.
[s] From V3.0.8: This parameter
makes it possible to extend the set
braking distance for the negative
direction by an amount proportional
to the actual velocity.
[s] From V3.0.8: This parameter
makes it possible to extend the set
braking distance for the positive
direction by an amount proportional
to the actual velocity.
[mm] From V3.0.8: Braking
distance: If bAsymetricalTargeting
is TRUE, at this negative distance
from the target, active profile-
controlled control value generation
ceases; optionally a standstill
position controller or a different
mechanism that applies at target is
activated.
[mm] From V3.0.8: Braking
distance: At this non-direction-
dependent or (if
bAsymetricalTargeting is TRUE)
positive distance from the target,
active profile-controlled control
value generation ceases; optionally
a standstill position controller or a
different mechanism that applies at
target is activated.
[s] If this parameter is set to a
value greater than 0, the control
value generator observes a delay
time between the rising edge at
ST_TcPlcDeviceOutput.bBrakeOff
and the start of the acceleration
phase.
[s] If this parameter is set to a
value greater than 0, the control
value generator observes a delay
time between the end of the active
profile generation and the falling
edge at ST_TcPlcDeviceOutput
[} 153].bBrakeOff.

TF5810

Version: 1.8.3

133

PLCopen Motion Control

Name
fBrakeSafetyDelay

Type
LREAL

fCreepDistanceM

LREAL

fCreepDistanceP

LREAL

fCreepSpeedM

LREAL

fCreepSpeedP

LREAL

fCustomerData

ARRAY

fCycletime

LREAL

fCylinder_ArreaA

LREAL

Description
[s] If this parameter is set to a
value greater than 0, the control
value generator at the falling edge
at ST_TcPlcDeviceOutput
[} 153].bBrakeOff observes a delay
time between the end of an active
profile generation and the rising
edge of the next motion command.
[mm] From V3.0.8: If
bAsymetricalTargeting is TRUE,
fCreepSpeedM is used as the
control value from this negative
distance to the target for the last
phase of the profile-controlled
control value generation.
[mm] From V3.0.8: From this non-
direction-dependent or (with
bAsymetricalTargeting = TRUE)
positive distance to the target,
fCreepSpeedP is used as the
control value for the last phase of
profile-controlled control value
generation.
[mm/s] From V3.0.8: If
bAsymetricalTargeting is TRUE
and the direction of movement is
negative, this velocity is used for
the last phase of the profile-
controlled control value generation.
[mm/s] From V3.0.8: This velocity
is used, in non-direction-dependent
mode, or (if bAsymetricalTargeting
is TRUE) if the direction of
movement is positive, for the last
phase of the profile-controlled
control value generation.
20 LREAL parameters are
available for use by the application,
as required. They are loaded and
stored together with the other axis
parameters. Library function blocks
do not use these parameters
independently, by the application
can instruct to use them based on
the type of call.
[s] The cycle time of the PLC task,
from which the library function
blocks are called. This value is
determined automatically by an
MC_AxUtiStandardInit_BkPlcMc
[} 254]() function block and may be
used but not be changed by the
application.
[mm2] The active area of the
cylinder, which is under pressure
during a motion in positive
direction.

134

Version: 1.8.3

TF5810

Name
fCylinder_ArreaB

fCylinder_Mass
fCylinder_Stroke
fDec

fDrive_IncInterpolation

Type
LREAL

LREAL
LREAL
LREAL

LREAL

fDrive_IncWeighting

LREAL

fEmergencyRamp

LREAL

fEnc_BaseDistance

LREAL

fEnc_DefaultHomePosition

LREAL

fEnc_IncInterpolation

LREAL

fEnc_IncWeighting

LREAL

fEnc_ModuloBase
fEnc_PotiRgToRl

LREAL
LREAL

fEnc_RefIndexVelo

LREAL

PLCopen Motion Control

Description
[mm2] The active area of the
cylinder, which is under pressure
during a motion in negative
direction.
reserved.
reserved.
fDec: [mm/s2] The absolute
deceleration limitation of the axis.
This parameter is used in some
output devices for internal
conversion of the velocity control
value.
This parameter is used in some
output devices for internal
conversion of the velocity control
value.
[s] This parameter specifies the
time required for deceleration from
fRefVelo to standstill. It is used by
different control value generators in
response to unscheduled
emergency stop requests (lack of
controller enable, fault condition,
function block call).
[mm] This parameter is used for
the evaluation of encoders with
distance-coded zero marks.
[mm] This parameter can be used
to store a position, which can be
transferred as reference position to
an MC_Home_BkPlcMc [} 68]()
function block. If homing is
triggered by the PlcMcManager,
the value stored here is used in this
way. If this is also intended to be
the case if homing is triggered by
the PLC application, this parameter
should be transferred when the
used function block is called.
[mm/n] This parameter specifies
the resolution with which the actual
position of the axis is determined.
[1] This parameter specifies the
resolution with which the actual
position of the axis is determined.

[1] It is used by some function
blocks for linearization of simple
potentiometer displacement
transducer, which are subject to
load from the input resistance of
the interface electronics.
[1] This parameter specifies the
control value as a proportion of
fRefVelo, which is output during a
search for the reference index
(cam) during homing.

TF5810

Version: 1.8.3

135

PLCopen Motion Control

Name
fEnc_RefSyncVelo

Type
LREAL

fEnc_ZeroShift

fJogVeloFast

fJogVeloSlow

fFeedForward
fAccFeedForward

fLagAmp

fLagAmpDp

fLagAmpDx

fLagAmpTi

fLagAmpOutL

fLagAmpWuL

fLagAmpTd

LREAL

LREAL

LREAL

LREAL
LREAL

LREAL

LREAL

LREAL

LREAL

LREAL

LREAL

LREAL

fLagAmpTdd

LREAL

fLagAmpCfb_kV

LREAL

Description
81] This parameter specifies the
control value as a proportion of
fRefVelo, which is output during a
search for the reference pulse
(sync pulse, zero pulse) during
homing.
[mm] This parameter shifts the zero
point of the actual value
determination of the axis.
[mm/s] Set velocity for fast manual
travel.
[mm/s] Set velocity for slow manual
travel.

[s] The optional acceleration pre-
control of the axis.
[mm/s per mm → 1/s] The Kp
amplification of the standstill
position controller.
[mm] In preparation: The response
window of the extended standstill
position controller.
[mm] In preparation: The response
window of the standstill position
controller.
In preparation: The integration time
of the standstill position controller.
In preparation: The output limit of
the standstill position controller.
In preparation: The limit of the I
part standstill position controller.
[1] Optional: Rate time of the
differential part of the position
controller.
This parameter is only used by
MC_AxRtPosPiControllerEx_BkP
lcMc().
[s] Optional: Damping time of the
differential part of the position
controller.

This parameter is only used by
MC_AxRtPosPiControllerEx_BkP
lcMc().
V: : [1] Optional: Weighting factor
of the actual velocity activation in
the condition feedback of the
position controller.

This parameter is only used by
MC_AxRtPosPiControllerEx_BkP
lcMc().

136

Version: 1.8.3

TF5810

Name
fLagAmpCfb_tV

Type
LREAL

fLagAmpCfb_kA

LREAL

fLagAmpCfb_tA

LREAL

fLagAmpCfb_kF

LREAL

fLagAmpCfb_tF

LREAL

fMaxAcc

fMaxDec

LREAL

LREAL

fMaxDynamicLag

LREAL

fMaxJerk

LREAL

fMaxLag

fMaxLagFilter

LREAL

LREAL

PLCopen Motion Control

Description
[1] Optional: Filter time of the
actual velocity activation in the
condition feedback of the position
controller.

This parameter is only used by
MC_AxRtPosPiControllerEx_BkP
lcMc().
[1] Optional: Weighting factor of the
actual acceleration activation in the
condition feedback of the position
controller.

This parameter is only used by
MC_AxRtPosPiControllerEx_BkP
lcMc().
[1] Optional: Filter time of the
actual acceleration activation in the
condition feedback of the position
controller.

This parameter is only used by
MC_AxRtPosPiControllerEx_BkP
lcMc().
[1] Optional: Weighting factor of the
actual force activation in the
condition feedback of the position
controller.

This parameter is only used by
MC_AxRtPosPiControllerEx_BkP
lcMc().
[1] Optional: Filter time of the
actual force activation in the
condition feedback of the position
controller.
[mm/s2] The axis acceleration
limitation applicable to the function
blocks. This value is limited to fAcc.
[mm/s2] The axis deceleration
limitation applicable to the function
blocks. This value is limited to
fDec.
[s] This parameter specifies one of
the limit values for the lag
monitoring.
[mm/s3] The axis jerk limitation
applicable to the function blocks.
This value is used if
iTcMc_ProfileJerkBased is set as
profile type.
[mm] This parameter specifies one
of the limit values for the lag
monitoring.
[s] This parameter specifies one of
the limit values for the lag
monitoring.

TF5810

Version: 1.8.3

137

PLCopen Motion Control

Name
fMaxVelo

fMonPositionRange

fMonTargetFilter

fMonTargetRange

fPEH_Timeout

fRefVelo

fReposDistance

fSoftEndMax

fSoftEndMin

fStartAccDistance

fStartRamp

Type
LREAL

LREAL

LREAL

LREAL

LREAL

LREAL

LREAL

LREAL

LREAL

LREAL

LREAL

fStopRamp

LREAL

fTargetClamping

LREAL

fVeloAmp

fVeloAmpDx

fVeloAmpTi

fVeloAmpOutL

LREAL

LREAL

LREAL

LREAL

Description
[mm/s] The maximum velocity that
can be used by function blocks. If a
function block tries to use a higher
value, the value is generally limited
accordingly without an error
message.
This parameter is limited to
fRefVelo.
[mm] This parameter is used for
target window monitoring.
[s] This parameter is used for
target window monitoring.
[mm] This parameter is used for
target window monitoring.
[s] This parameter specifies the
limit value for PEH monitoring.
[mm/s] This parameter specifies
the maximum absolute axis
velocity.
[mm] If this parameter is greater
than 0 and the axis has moved
beyond the target by more than this
distance, target positioning is
automatically applied again.
[mm] The upper (positive) software
limit switch.
[mm] The lower (negative) software
limit switch.
obsolete, will be removed in the
near future.
[s] This parameter specifies the
time required in profile type
iTcMc_ProfileTimeRamp to
accelerate to fRefVelo.
[s] This parameter specifies the
time required for deceleration from
fRefVelo to standstill. It is used in
profile type
iTcMc_ProfileTimeRamp for the
target approach, and also by
control value generators in
response to unscheduled stop
requests (lack of feed enable, fault
condition, function block call).
[v] If this parameter is set to a
value greater than zero, this control
value is output with the correct sign
when a target is reached. A
position control is suppressed.
The Kp gain of the lower-level
velocity controller.
The response window of the lower-
level velocity controller.
The integration time of the lower-
level velocity controller.
The output limitation of the lower-
level velocity controller.

138

Version: 1.8.3

TF5810

Name
fVeloAmpWuL

fValve_BendPointOutput

Type
LREAL

LREAL

fValve_BendPointVelo

LREAL

fValve_OverlapCompM

LREAL

fValve_OverlapCompP

LREAL

fValve_ResponseTime

LREAL

fZeroCompensation

LREAL

nEnc_OverrunMask
nEnc_PositionMask
nEnc_ZeroSwap
nDigInReversed
nCycleDivider
nDrive_Type

nEnc_HomingType

DWORD
DWORD
DINT
DINT
INT
E_TcMcDriveType

E_TcMcHomingType

nEnc_Type

nJerkEnabled

E_TcMcEncoderType

WORD

nProfileType

E_TcMcProfileType

nControllerType

WORD

nOverlapDefMode

WORD

PLCopen Motion Control

Description
Limitation of the I part of the lower-
level velocity controller.
[1] In valves with a bend in the
characteristic curve, this parameter
can be used for simple
linearization.
[1] In valves with a bend in the
characteristic curve, this parameter
can be used for simple
linearization.
[1] From V3.0.8: Compensation (if
bAsymetricalTargeting = TRUE) for
of a valve overlap used for the
negative direction.
[1] From V3.0.8: Compensation of
a non-direction-dependent valve
overlap or (if
bAsymetricalTargeting = TRUE) a
valve overlap used for the positive
direction.
[s] This parameter can be used for
dead time compensation of the
actuator.
[V] This parameter can be used to
compensate an analog offset of the
velocity output.

reserved.

reserved.

Specifies the drive type [} 94].
Used to specify the referencing
method, which an
MC_Home_BkPlcMc() [} 68]
function block uses if
MC_DefaultHomingMode_BkPlcMc
[} 125] is transferred as
HomingMode.

Specifies the encoder type [} 98].
This bit mask determines at which
transitions in the profile jerk
limitation is to be applied. This
value is used if
iTcMc_ProfileJerkBased is set as
profile type.

Specifies the control value
generator [} 118].
This parameter is set automatically
by the called position controller. It
is not saved.
reserved.

TF5810

Version: 1.8.3

139

PLCopen Motion Control

Name
bAsymetricalTargeting

Type
BOOL

bDrive_AbsoluteOutput

BOOL

bDrive_DefaultPowerOk

BOOL

bDrive_Reversed

BOOL

bEnableAutoBrakeDistance

BOOL

bEnableControlLoopOnFault

BOOL

bEnc_RefIndexPositive

BOOL

bEnc_RefSyncPositive

BOOL

bEnc_Reversed

bMaxLagEna

bPEH_Enable

bPosCtrlAccEna

bSoftEndMaxEna

bSoftEndMinEna

bTimeBased

BOOL

BOOL

BOOL

BOOL

BOOL

BOOL

BOOL

Description
From V3.0.8: If this parameter is
TRUE, direction-dependent
parameters take effect during
target approach and overlap
compensation.
If this parameter is set to TRUE,
control values are always output
positively, regardless of the
direction.
If this parameter is set, the
PowerOk feedback in the
ST_TcPlcDeviceInput [} 149]
structure of the axis is ignored.
If this parameter is set, the control
value output is negated.
If this parameter is TRUE,
fCreepDistanceM and
fCreepDistanceP are calculated
automatically from fCreepSpeedM
or fCreepSpeedP and fLagAmp.
If this parameter is TRUE, the
standstill position controller of the
axis also becomes active in the
event of an error. Requirement: Its
parameters are suitable for this,
and the axis is in a suitable state.
If this parameter is set, while
searching for the reference index
(cam) during homing a positive
control value is output, otherwise a
negative value.
If this parameter is set, while
searching for the reference pulse
(sync pulse, zero pulse) during
homing a positive control value is
output, otherwise a negative value.
If this parameter is set, the actual
position value is evaluated in
negated form.
This parameter activates lag
monitoring.
This parameter activates the PEH
monitoring.
obsolete, will be removed in the
near future.
This parameter activates the upper
software limit switch.
This parameter activates the lower
software limit switch.
If this parameter is TRUE, the
profile calculations are timer-
controlled. The position controller is
always active.

140

Version: 1.8.3

TF5810

Name
bLinTabAvailable

Type
BOOL

bEnc_NoUpLoad

BOOL

bDrive_NoUpLoad

BOOL

bDriveIsHybrid

BOOL

bAlignedStart

BOOL

bEncModuloMode
stHybrid

BOOL
ST_TcHybridAxParam

stPctrl

ST_TcPctrlParam

PLCopen Motion Control

Description
TRUE here means that each
pointer was associated with a
linearization table during
initialization, which contains a
successfully determined
characteristic curve.
If this parameter is set, no
parameters are read from the
device, even in configurations with
fieldbus encoders.
If this parameter is set, no
parameters are read from the
device, even in configurations with
fieldbus drives and valves.
This parameter is used to identify a
servo-electric/hydraulic hybrid axis.
The extended parameters in
stHybrid take effect and appear in
the PlcMcManager.
From V3.x.y: If this parameter is
TRUE, a jump of the output is
avoided when starting from a lag
error.
reserved.
This structure pools parameters for
hybrid electro/hydraulic axes.
This structure contains parameters
that can be used for a force or
pressure controller.

fBrakeDistance: [mm] Up to V3.0.7: Braking distance: At this non-direction-dependent positive distance
from the target, active profile-controlled control value generation ceases; optionally a standstill position
controller or a different mechanism that applies at target is activated.

fBrakeDeadTime:[s] Up to V3.0.7: This parameter allows to extend the set braking distance with a portion
proportional to the actual speed.

fCreepSpeed:[mm/s] Up to V3.0.7: This velocity is used non-direction-dependent for the last phase of
profile-controlled control value generation.

fCreepDistance:[mm] Up to V3.0.7: From this non-direction-dependent distance to the target, fCreepSpeed
is used as the control value for the last phase of profile-controlled control value generation.

This parameter is only used by MC_AxRtPosPiControllerEx_BkPlcMc().

fValve_OverlapComp:[1] Up to V3.0.7: Compensation of a non-direction-dependent valve overlap.

See Commissioning [} 380] for more information about axis commissioning.

4.3.25

ST_TcHydAxRtData

Available from version 3.0

The variables in this structure indicate the runtime state of the axis.

TF5810

Version: 1.8.3

141

PLCopen Motion Control

The order of the data is not guaranteed.

Syntax
TYPE ST_TcHydAxRtData :
(* last modification: 02.07.2018 *)
STRUCT
(*-------------------------------*)
fActForce:           LREAL := 0.0;
fActiveOverlap:      LREAL := 0.0;
fActPos:             LREAL := 0.0;
fActPosDelta:        LREAL := 0.0;
fActPosOffset:       LREAL := 0.0;
fActPressure:        LREAL := 0.0;
fActPressureA:       LREAL := 0.0;
fActPressureB:       LREAL := 0.0;
fActVelo:            LREAL := 0.0;
fBrakeOffTimer:      LREAL := 0.0;
fBrakeOnTimer:       LREAL := 0.0;
fBrakeSafetyTimer:   LREAL := 0.0;
fClampingOutput:     LREAL := 0.0;
fDestAcc:            LREAL := 0.0;
fDestCreepDistanceM: LREAL := 0.0;
fDestCreepDistanceP: LREAL := 0.0;
fDestCreepSpeedM:    LREAL := 0.0;
fDestCreepSpeedP:    LREAL := 0.0;
fDestDec:            LREAL := 0.0;
fDestJerk:           LREAL := 0.0;
fDestPos:            LREAL := 0.0;
fDestRampEnd:        LREAL := 0.0;
fDestSpeed:          LREAL := 0.0;
fDistanceToTarget:   LREAL := 0.0;
fEnc_RefShift:       LREAL := 0.0;
fEnc_ZeroSwap:       LREAL := 0.0;
fGearActive:         LREAL := 0.0;
fGearSetting:        LREAL := 0.0;
fLagCtrlOutput:      LREAL := 0.0;
fLatchedPos:         LREAL := 0.0;
fOilRequirred_A:     LREAL := 0.0;
fOilRequirred_B:     LREAL := 0.0;
fOilUsed_A:          LREAL := 0.0;
fOilUsed_B:          LREAL := 0.0;
fOutput:             LREAL := 0.0;
fOverride:           LREAL := 1.0;
fParamAccTime:       LREAL := 0.0;
fPosError:           LREAL := 0.0;
fSetAcc:             LREAL := 0.0;
fSetPos:             LREAL := 0.0;
fSetPressure:        LREAL := 0.0;
fSetSpeed:           LREAL := 0.0;
fSetSpeedOld:        LREAL := 0.0;
fSetVelo:            LREAL := 0.0;
fStartPos:           LREAL := 0.0;
fStartRamp:          LREAL := 0.0;
fStartRampAnchor:    LREAL := 0.0;
fSupplyPressure:     LREAL := 0.0;
fTargetPos:          LREAL := 0.0;
fTimerPEH:           LREAL := 0.0;
fTimerTPM:           LREAL := 0.0;
fValvePressure:      LREAL := 0.0;
fVeloError:          LREAL := 0.0;
fBlockDetectDelay: LREAL := 2.0;
(*------------------------------------------------------*)
nAxisState:          DWORD := 0;
nCalibrationState:   DWORD := 0;
nDeCtrlDWord:        DWORD := 0;
nErrorCode:          DWORD := 0;
nStateDWord:         DWORD := 0;
udiAmpErrorCode:     UDINT;
(*------------------------------------------------------*)
iCurrentStep: E_TcMcCurrentStep;
wEncErrMask:         WORD:=0;
wEncErrMaskInv:      WORD:=0;
nDrvWcCount:         INT:=0;
(**)
nEncWcCount:         INT:=0;

142

Version: 1.8.3

TF5810

PLCopen Motion Control

nDrvDeviceState:     UINT:=0;
nEncDeviceState:     INT:=0;
(*------------------------------------------------------*)
bActPosCams:         BYTE := 0;
bBrakeOff:           BOOL := FALSE;
bBrakeOffInverted:   BOOL := FALSE;
bControllable:       BOOL := FALSE;
bCountedCycles:      BYTE := 1;
bCycleCounter:       BYTE := 0;
bDriveResponse:      BOOL := FALSE;
bEncDoLatch:         BOOL := FALSE;
(**)
bEncoderResponse:    BOOL := FALSE;
bEncLatchValid:      BOOL := FALSE;
bLocked_Estop:       BOOL := FALSE;
bParamsUnsave:       BOOL := FALSE;
bReloadParams:       BOOL := FALSE;
bTargeting:          BOOL := FALSE;
bUnalignedOverlap:   BOOL := FALSE;
bActPosOffsetEnable: BOOL := FALSE; (* starting with 09.03.2015 *)
(**)
bDriveStartup:       BOOL := FALSE;
bEncAlignRefShift:   BOOL := FALSE;
bDrvWcsError:        BOOL := FALSE;
bEncWcsError:        BOOL := FALSE;
bFirstWcs:           BOOL := FALSE;
bChangeCount:        BYTE := 0;
bStartAutoIdent:     BOOL := FALSE;
bParamFileComplete:  BOOL := FALSE;
(*------------------------------------------------------*)
pMasterRtData:       POINTER TO BYTE;
pMasterParam:        POINTER TO BYTE;
(*------------------------------------------------------*)
udiSercDeviceID:     UDINT := 0;
uiSercBoxAddr:       UINT := 0;
uiSercPort:          UINT := 0;
(*------------------------------------------------------*)
stPosCtrlr: stbkplcinternal_cplxctrl;
stVeloCtrlr: stbkplcinternal_cplxctrl;
(*------------------------------------------------------*)
sTopBlockName:       STRING(87) := '';

stHybrid:           ST_TcHybridAxRtData;
(*------------------------------------------------------*)
END_STRUCT
END_TYPE

TF5810

Version: 1.8.3

143

PLCopen Motion Control

Parameter

144

Version: 1.8.3

TF5810

Name
fActForce

fActiveOverlap

fActPos

fActPosDelta

fActPosOffset

Type
LREAL

LREAL

LREAL

LREAL

LREAL

fActPressure

LREAL

fActPressureA

LREAL

fActPressureB

LREAL

fActVelo

fBrakeOffTimer
fBrakeOnTimer
fBrakeSafetyTimer
fClampingOutput
fDestAcc

fDestCreepDistance
fDestCreepDistanceM

LREAL

LREAL
LREAL
LREAL
LREAL
LREAL

LREAL
LREAL

fDestCreepDistanceP

LREAL

fDestCreepSpeed
fDestCreepSpeedM

LREAL
LREAL

fDestCreepSpeedP

LREAL

PLCopen Motion Control

Description
[N, kN] Actual force of the cylinder. This value is
usually determined by a function block for
acquisition of actual force or pressure values [} 20].
[1] The current output of the overlap compensation.
An output variable of the profile generators.
[mm] The current actual position of the axis. This
value is usually determined by an encoder function
block.
[mm] The change of the actual position relative to
the previous cycle.
[mm] The offset used to influence the actual value.
If bActPosOffsetEnable is TRUE, this offset is
added to fActPos. If fActPosOffset changes,
fActVelo is unaffected.

If bActPosOffsetEnable is TRUE, fActPosOffset
takes effect immediately and without ramp.

Note the information. [} 149]

Example: If the reference position is 100.0 mm and
the offset is 1.0 mm, the actual position at the point
of the zero pulse is set to 101.0 mm. If influencing
is subsequently disabled or set to 0.0, the actual
position at the point of the zero pulse shows the
value 100.0 mm, just like it would have done during
homing without influencing.
[bar] Actual pressure in the cylinder. This value is
usually determined by a function block for
acquisition of actual force or pressure values [} 20].
[bar] Actual pressure on the A-side of the cylinder.
This value is usually determined by a function
block for acquisition of actual force or pressure
values [} 20].
[bar] Actual pressure on the B-side of the cylinder.
This value is usually determined by a function
block for acquisition of actual force or pressure
values [} 20].
[mm/s] The current actual velocity of the axis. This
value is usually determined by an encoder function
block.

[V] An output variable of the profile generators.
[mm/s2] The acceleration specified by the current
or last executed motion command.
[mm] Up to V3.0.7: The creep distance.
[mm] Up to V3.0.8: The creep distance in negative
direction.
[mm] Up to V3.0.8: The creep distance in positive
direction.
[mm/s] Up to V3.0.7: The creep speed.
From V3.0.8: The creep speed in negative
direction.
[mm/s] From V3.0.8: The creep speed in positive
direction.

TF5810

Version: 1.8.3

145

PLCopen Motion Control

Name
fDestDec

fDestJerk

fDestPos
fDestRampEnd
fDestSpeed

Type
LREAL

LREAL

LREAL
LREAL
LREAL

fDistanceToTarget

LREAL

fEnc_RefShift

LREAL

fEnc_ZeroSwap
fGearActive
fGearSetting
fLagCtrlOutput

fLatchedPos

fOilRequirred_A

fOilRequirred_B

fOilUsed_A

fOilUsed_B

fOutput

fOverride
fParamAccTime
fPosError
fSetAcc

fSetPos

fSetPressure

fSetSpeed

fSetSpeedOld
fSetVelo
fStartPos

LREAL
LREAL
LREAL
LREAL

LREAL

LREAL

LREAL

LREAL

LREAL

LREAL

LREAL
LREAL
LREAL
LREAL

LREAL

LREAL

LREAL

LREAL
LREAL
LREAL

Description
[mm/s2] The deceleration specified by the current
or last executed motion command.
[mm/s3] The jerk specified by the current or last
executed motion command.
[mm] The currently active target position.

[mm/s] The velocity specified by the current or last
executed motion command.
[mm] The current remaining distance of the axis.
This value is usually determined by a generator
function block.
[mm] The offset between the converted (perhaps
internal extended) counter value of an incremental
encoder input terminal and the actual position of
the axis. This offset is determined through homing,
e.g. with an MC_Home_BkPlcMc [} 68] function
block, or manipulated with an
MC_SetPosition_BkPlcMc [} 43] function block.

[1] The normalized output of the position controller.
An output variable of the profile generators.
[mm] The position (taking into account current
offsets) at which homing took place or where the
components of the actual value acquisition
(encoder, I/O electronics) were switched on.
[l/min] The oil consumption on the A-side,
calculated taking into account the set velocity.
[l/min] The oil consumption on the B-side,
calculated taking into account the set velocity.
[l/min] The oil consumption on the A-side,
calculated taking into account the actual velocity.
[l/min] The oil consumption on the B-side,
calculated taking into account the actual velocity.
[1] The control value to be output. This variable is
used for communication between the
MC_AxRtFinish_BkPlcMc [} 246] and
MC_AxRtDrive_BkPlcMc [} 187] function blocks.
[1] The current axis velocity override.

[mm] The current position error of the axis.
[mm/s2] The current acceleration control value. An
output variable of the profile generators.
[mm] The current position command value of the
axis.
[bar] The setpoint for an optional pressure or force
control must be stored here.
[mm/s] The normalized set velocity of the axis. An
output variable of the profile generators.

[mm] The start position of the current or last
processed motion command.

146

Version: 1.8.3

TF5810

Name
fStartRamp
fStartRampAnchor
fSupplyPressure

fTargetPos

fTimerPEH
fTimerTPM
fValvePressure

fVeloError
fBlockDetectDelay

Type
LREAL
LREAL
LREAL

LREAL

LREAL
LREAL
LREAL

LREAL
LREAL

nAxisState
nCalibrationState
nDeCtrlDWord

nErrorCode

nStateDWord

udiAmpErrorCode
iCurrentStep

wEncErrMask
wEncErrMaskInv
nDrvWcCount
nEncWcCount
nDrvDeviceState
nEncDeviceState
bActPosCams

bBrakeOff

bBrakeOffInverted
bControllable
bCountedCycles
bCycleCounter
bDriveResponse
bEncDoLatch

DWORD
DWORD
DWORD

DWORD

DWORD

UDINT
E_TcMcCurrentStep

WORD
WORD
INT
INT
UINT
INT
BYTE

BOOL

BOOL
BOOL
BYTE
BYTE
BOOL
BOOL

PLCopen Motion Control

Description

[bar] Supply pressure. This value is usually
determined by a function block for acquisition of
actual force or pressure values [} 20].
[mm] The target position specified by the current or
last processed motion command.

[bar] Pressure drop at the valve. This value is
usually determined by a function block for
acquisition of actual force or pressure values [} 20].

[s] The delay time for the detection of the function
block during homing on block. This value is
initialized with 2.0 seconds to reflect the default
behavior of previous versions. If a different time is
required, it must be updated before homing
commences. If a value of less than the cycle time
is detected when homing commences, the default
value of 2.0 seconds is entered automatically. This
value is not saved as a parameter. This variable
has been available under TwinCAT 2 in V3.0.41
from 12 October 2017.
The motion state of the axis.
The current homing state.

The control signals [} 339] of the axis.

The current ErrorCode [} 339] of the axis.

The state signals [} 338] of the axis.

The internal state of the control value generators.
Values from E_TcMcCurrentStep [} 90].

The current position cam of the axis. This value is
only used, if iTcMc_EncoderDigCam is set as
encoder type.
The control signal for an external brake. An output
variable of the profile generators.
The inverted bBrakeOff signal.

This signal is used for communication by the
MC_Home_BkPlcMc [} 68] and
MC_AxRtEncoder_BkPlcMc [} 198] function blocks
of the axis during homing.

TF5810

Version: 1.8.3

147

PLCopen Motion Control

Name
bEncoderResponse
bEncLatchValid

Type
BOOL
BOOL

bLocked_Estop

BOOL

bParamsUnsave

BOOL

bReloadParams
bTargeting
bUnalignedOverlap

BOOL
BOOL
BOOL

bActPosOffsetEnable

BOOL

bDriveStartup
bEncAlignRefShift
bDrvWcsError
bEncWcsError
bFirstWcs
bChangeCount

bStartAutoIdent
bParamFileComplete

BOOL
BOOL
BOOL
BOOL
BOOL
BYTE

BOOL
BOOL

pMasterRtData
pMasterParam
udiSercDeviceID
uiSercBoxAddr
uiSercPort
stPosCtrlr
stVeloCtrlr
sTopBlockName

POINTER TO BYTE
POINTER TO BYTE
UDINT
UINT
UINT
stbkplcinternal_cplxctrl
stbkplcinternal_cplxctrl
STRING

stHybrid

ST_TcHybridAxRtData

Description

This signal is used for communication by the
MC_Home_BkPlcMc [} 68] and
MC_AxRtEncoder_BkPlcMc [} 198] function blocks
of the axis during homing.
A TRUE in this variable prevents the control value
generators from exiting the state
iTcHydStateEmergencyBreak / McState_Errorstop,
despite the fact that the drive outputs are reduced
to 0. Used by MC_EmergencyStop_BkPlcMc [} 57]
and MC_ImediateStop_BkPlcMc [} 72].

The function blocks MC_WriteParameter_BkPlcMc
[} 48] and MC_WriteBoolParameter_BkPlcMc [} 46]
set this flag if they change a parameter value. An
MC_AxParamSave_BkPlcMc [} 288] function block
clears the flag when the parameters are
successfully saved. In online mode of the
PlcMcManager [} 371], this flag is used for the
state display.

The characteristic of the overlap compensation is
defined here.
A TRUE in this variable activates actual value
influencing. See also under fActPosOffset.

reserved.

This value is incremented with each parameter
change.

This flag is set if a corresponding identifier was
found at the end of the file when the parameters
were loaded and the CRC check was successful.

Most of the library function blocks called directly by
application enter a debug ID here.
Extended status data for servo-electric/hydraulic
hybrid axes.

148

Version: 1.8.3

TF5810

PLCopen Motion Control

Information for fActPosOffset

• If actual value influencing is active during homing, bActPosOffset is taken into account when the

actual position is set.

• This function is only realized for the following encoder types: iTcMc_EncoderCoE_DS406,

iTcMc_EncoderEL3255, iTcMc_EncoderSim, iTcMc_EncoderEL5101, iTcMc_EncoderKL5101,
iTcMc_EncoderKL5111, iTcMc_EncoderEL5001, iTcMc_EncoderKL5001,
iTcMc_EncoderKL3002, iTcMc_EncoderEL3102, iTcMc_EncoderKL3042,
iTcMc_EncoderKL3062, iTcMc_EncoderEL3142, iTcMc_EncoderEM8908_A,
iTcMc_EncoderEL3162, iTcMc_EncoderKL3162.

• If one of the types listed is set for an I/O device that is compatible with one of these types, the

function described is also realized.

All other elements of this structure are reserved for internal use. They are not guaranteed and must
not be used or modified by the application.

4.3.26

ST_TcMcAuxDataLabels

Available from version 3.0

This structure is used for storing the label texts for the customer-specific axis parameters. A structure of this
type can be linked with the axis through an MC_AxUtiStandardInit_BkPlcMc [} 254] function block via a
pointer in the AXIS_REF_BkPlcMc [} 86] structure.

Syntax
TYPE ST_TcMcAuxDataLabels:
STRUCT
    stLabel:        ARRAY [1..20] OF STRING(20);
END_STRUCT
END_TYPE

Parameter

Name
stLabel

Type
ARRAY

Description
The label texts

4.3.27

ST_TcPlcDeviceInput

Available from version 3.0

This structure contains the input image variables of an axis.

Syntax
TYPE ST_TcPlcDeviceInput :
STRUCT
    uiCount:         UINT:=0;
    uiLatch:         UINT:=0;
    usiStatus:       USINT:=0;

    uiPZDL_RegDaten: UINT:=0;
    uiPZDH:          UINT:=0;
    usiRegStatus:    USINT:=0;

    udiCount:        UDINT:=0;
    uiStatus:        UINT:=0;

TF5810

Version: 1.8.3

149

PLCopen Motion Control

    bTerminalState:  BYTE:=0;
    uiTerminalData:  WORD:=0;
    uiTerminalState2:WORD:=0;

    bDigInA:         BOOL:=FALSE;
    bDigInB:         BOOL:=FALSE;

    bDigCamMM:       BOOL:=FALSE;
    bDigCamM:        BOOL:=FALSE;
    bDigCamP:        BOOL:=FALSE;
    bDigCamPP:       BOOL:=FALSE;

    DriveError:      UDINT:=0;
    ActualPos:       ARRAY [0..1] OF UINT:=0;
    DriveState:      ARRAY [0..3] OF BYTE:=0;

    S_iReserve:      INT:=0;
    S_DiReserve:     ARRAY [1..9] OF DINT:=0;

    CiA_Reserve:     ARRAY [1..8] OF UINT:=0;

    bPowerOk:        BOOL:=FALSE;
    bEnAck:          BOOL:=FALSE;

    wDriveDevState:  WORD:=0;
    wDriveWcState:   BYTE:=0;
    wEncDevState:    WORD:=0;
    wEncWcState:     BYTE:=0;
    uiDriveBoxState: UINT:=0;
    uiEncBoxState:   UINT:=0;

    sEncAdsAddr:     ST_TcPlcAdsAddr;
    nEncAdsChannel:  BYTE:=0;
    sDrvAdsAddr:     ST_TcPlcAdsAddr;
    nDrvAdsChannel:  BYTE:=0;

    nReserve:        ARRAY [1..20] OF BYTE;
END_STRUCT
END_TYPE

150

Version: 1.8.3

TF5810

Parameter

PLCopen Motion Control

TF5810

Version: 1.8.3

151

PLCopen Motion Control

Name
uiCount

Type
UINT

uiLatch

UINT

usiStatus

USINT

uiPZDL_RegDaten

UINT

uiPZDH

usiRegStatus

udiCount

uiStatus

UINT

USINT

UDINT

UINT

bTerminalState

BYTE

uiTerminalData
uiTerminalState2

WORD
WORD

bDigInA

bDigInB

bDigCamMM

bDigCamM

bDigCamP

bDigCamPP

BOOL

BOOL

BOOL

BOOL

BOOL

BOOL

DriveError

UDINT

ActualPos

ARRAY

Description
Used for position detection. Used for
iTcMc_EncoderEL3102, iTcMc_EncoderEL3142,
iTcMc_EncoderEL5101, iTcMc_EncoderKL2521,
iTcMc_EncoderKL2531, iTcMc_EncoderKL2541,
iTcMc_EncoderKL3002, iTcMc_EncoderKL3042,
iTcMc_EncoderKL3062, iTcMc_EncoderKL3162,
iTcMc_EncoderKL5101, iTcMc_EncoderKL5111,
iTcMc_EncoderM2510, iTcMc_EncoderM3120,
iTcMc_DriveKL2531, iTcMc_DriveKL2541.
Used for position detection. Used for
iTcMc_EncoderEL5101, iTcMc_EncoderKL5101,
iTcMc_EncoderKL5111.
Used for device state information. Used for
iTcMc_EncoderEL5101, iTcMc_EncoderKL3002,
iTcMc_EncoderKL3042, iTcMc_EncoderKL3062,
iTcMc_EncoderKL3162, iTcMc_EncoderKL5101,
iTcMc_EncoderKL5111, iTcMc_EncoderM3120.
Used for position detection and parameter
communication. Used for iTcMc_EncoderKL5001.
Used for position detection. Used for
iTcMc_EncoderKL5001.
Used for device state information. Used for
iTcMc_EncoderEL5001, iTcMc_EncoderKL5001.
Used for position detection. Used for
iTcMc_EncoderEL5001.
Used for device state information. Used for
iTcMc_EncoderAx2000_B110,
iTcMc_DriveAx2000_B110.
Used for parameter communication. Used for
iTcMc_EncoderKL2521, iTcMc_EncoderKL2531,
iTcMc_EncoderKL2541, iTcMc_DriveEL4132,
iTcMc_DriveKL2521, iTcMc_DriveKL2531,
iTcMc_DriveKL2541, iTcMc_DriveKL4032.
Reserved
Used for position detection. Used for
iTcMc_EncoderKL2541.
Used for position detection. Used for
iTcMc_EncoderDigIncrement.
Used for position detection. Used for
iTcMc_EncoderDigIncrement.
Used for position detection. Used for
iTcMc_EncoderDigCam.
Used for position detection. Used for
iTcMc_EncoderDigCam.
Used for position detection. Used for
iTcMc_EncoderDigCam.
Used for position detection. Used for
iTcMc_EncoderDigCam.
Used for device state information. Used for
iTcMc_EncoderAx2000_B200,
iTcMc_EncoderAx2000_B900.
Used for position detection. Used for
iTcMc_EncoderAx2000_B110,
iTcMc_EncoderAx2000_B200,
iTcMc_EncoderAx2000_B900.

152

Version: 1.8.3

TF5810

PLCopen Motion Control

Description
Used for device state information. Used for
iTcMc_EncoderAx2000_B200,
iTcMc_EncoderAx2000_B900.
Reserved
Reserved
Reserved
Optionally used for monitoring of a mains contactor.
Used for iTcMc_DriveAx2000_B110,
iTcMc_EncoderAx2000_B200,
iTcMc_EncoderAx2000_B900.
Reserved
Reserved
Used for monitoring the connection to the actuator. Used
for iTcMc_EncoderAx2000_B110,
iTcMc_DriveAx2000_B110.
Reserved
Used for monitoring the connection to the encoder. Used
for iTcMc_EncoderAx2000_B110,
iTcMc_DriveAx2000_B110, iTcMc_EncoderEL3102,
iTcMc_EncoderEL3142, iTcMc_EncoderEL5001,
iTcMc_EncoderEL5101.
Used for monitoring the connection to the actuator. Used
for iTcMc_DriveAx2000_B200,
iTcMc_DriveAx2000_B900.
Used for monitoring the connection to the encoder. Used
for iTcMc_EncoderAx2000_B200,
iTcMc_EncoderAx2000_B900.
Used for parameter communication. Used for
iTcMc_EncoderAx2000_B110,
iTcMc_DriveAx2000_B110, iTcMc_EncoderEL3102,
iTcMc_EncoderEL3142, iTcMc_EncoderEL5001,
iTcMc_EncoderEL5101.
Used for parameter communication. Used for
iTcMc_EncoderAx2000_B110,
iTcMc_DriveAx2000_B110.
Used for parameter communication. Used for
iTcMc_EncoderAx2000_B110,
iTcMc_DriveAx2000_B110.
Used for parameter communication. Used for
iTcMc_EncoderAx2000_B110,
iTcMc_DriveAx2000_B110.
Reserved

Name
DriveState

S_iReserve
S_DiReserve
CiA_Reserve
bPowerOk

bEnAck
wDriveDevState
wDriveWcState

wEncDevState
wEncWcState

Type
ARRAY

INT
ARRAY
ARRAY
BOOL

BOOL
WORD
BYTE

WORD
BYTE

uiDriveBoxState

UINT

uiEncBoxState

UINT

sEncAdsAddr

ST_TcPlcAdsAddr

nEncAdsChannel

BYTE

sDrvAdsAddr

ST_TcPlcAdsAddr

nDrvAdsChannel

BYTE

nReserve

ARRAY

4.3.28

ST_TcPlcDeviceOutput

Available from version 3.0

This structure contains the output image variables of an axis.

Syntax
TYPE ST_TcPlcDeviceOutput :
STRUCT
    nDacOut:          INT:=0;
    bDigOutAp:        BOOL:=FALSE;

TF5810

Version: 1.8.3

153

PLCopen Motion Control

    bDigOutAn:        BOOL:=FALSE;
    bDigOutBp:        BOOL:=FALSE;
    bDigOutBn:        BOOL:=FALSE;
    uiCount:          UINT:=0;
    uiDacOutA:        UINT:=0;
    uiDacOutB:        UINT:=0;
    bMovePos:         BOOL:=FALSE;
    bMoveNeg:         BOOL:=FALSE;
    bBrakeOff:        BOOL:=FALSE;
    bBrakeOffInverted:BOOL:=FALSE;
    DriveCtrl:        ARRAY [0..3] OF BYTE:=0;
    NominalVelo:      DINT:=0;
    uiDriveCtrl:      UINT:=0;
    S_iReserve:       ARRAY [1..2] OF INT:=0;
    S_DiReserve:      ARRAY [1..7] OF DINT:=0;
    CiA_Reserve:      ARRAY [1..7] OF UINT:=0;
    bPowerOn:         BOOL:=FALSE;
    bEnable:          BOOL:=FALSE;
    bEnablePos:       BOOL:=FALSE;
    bEnableNeg:       BOOL:=FALSE;
    nResetState:      BYTE:=0;
    usiCtrl:          USINT:=0;
    uiTerminalData:   WORD:=0;
    bTerminalCtrl:    BYTE:=0;
    uiTerminalCtrl2:  WORD:=0;
    nReserve:         ARRAY [1..20] OF BYTE;
END_STRUCT
END_TYPE

154

Version: 1.8.3

TF5810

Parameter

Name
nDacOut

bDigOutAp
bDigOutAn
bDigOutBp
bDigOutBn
uiCount
uiDacOutA

uiDacOutB
bMovePos
bMoveNeg
bBrakeOff
bBrakeOffInverted
DriveCtrl

NominalVelo

uiDriveCtrl

S_iReserve
S_DiReserve
CiA_Reserve
bPowerOn

bEnable
bEnablePos
bEnableNeg
nResetState
usiCtrl

Type
INT

BOOL
BOOL
BOOL
BOOL
UINT
UINT

UINT
BOOL
BOOL
BOOL
BOOL
ARRAY

DINT

UINT

ARRAY
ARRAY
ARRAY
BOOL

BOOL
BOOL
BOOL
BYTE
USINT

uiTerminalData

WORD

bTerminalCtrl

BYTE

uiTerminalCtrl2

WORD

nReserve

ARRAY

PLCopen Motion Control

Description
Used for control value outputs or parameter communication. Used for
iTcMc_EncoderKL2531, iTcMc_EncoderKL2541, iTcMc_DriveEL4132,
iTcMc_DriveKL2521, iTcMc_DriveKL2531, iTcMc_DriveKL2541,
iTcMc_DriveKL4032, iTcMc_DriveM2400_Dn.
Used for control value output. Used for iTcMc_DriveLowCostStepper.
Used for control value output. Used for iTcMc_DriveLowCostStepper.
Used for control value output. Used for iTcMc_DriveLowCostStepper.
Used for control value output. Used for iTcMc_DriveLowCostStepper.
Reserved
Used for control value output. Used for iTcMc_EncoderIx2512_1Coil,
iTcMc_EncoderIx2512_2Coil.
Used for control value output. Used for iTcMc_EncoderIx2512_2Coil.
Reserved
Reserved
Reserved
Reserved
Used for device control signals. Used for
iTcMc_EncoderAx2000_B200, iTcMc_DriveAx2000_B200,
iTcMc_EncoderAx2000_B900, iTcMc_DriveAx2000_B900.
Used for control value output. Used for iTcMc_DriveAx2000_B110,
iTcMc_EncoderAx2000_B200, iTcMc_EncoderAx2000_B900.
Used for device control signals. Used for
iTcMc_EncoderAx2000_B110, iTcMc_DriveAx2000_B110.
Reserved
Reserved
Reserved
Optionally used for controlling a mains contactor. Used for
iTcMc_DriveAx2000_B110, iTcMc_EncoderAx2000_B200,
iTcMc_EncoderAx2000_B900.
Reserved
Reserved
Reserved
Reserved
Used for device control signals or parameter communication. Used for
iTcMc_EncoderEL5101, iTcMc_EncoderKL3002,
iTcMc_EncoderKL3042, iTcMc_EncoderKL3062,
iTcMc_EncoderKL3162, iTcMc_EncoderKL5101,
iTcMc_EncoderKL5111, iTcMc_EncoderM3120.
Used for parameter communication. Used for iTcMc_EncoderKL2521,
iTcMc_EncoderKL5001, iTcMc_EncoderKL5101,
iTcMc_EncoderKL5111, iTcMc_DriveEL4132, iTcMc_DriveKL2521,
iTcMc_DriveKL4032.
Used for parameter communication. Used for iTcMc_EncoderKL2521,
iTcMc_EncoderKL2531, iTcMc_EncoderKL2541, iTcMc_DriveEL4132,
iTcMc_DriveKL2521, iTcMc_DriveKL2531, iTcMc_DriveKL2541,
iTcMc_DriveKL4032.
Used for device control signals. Used for iTcMc_EncoderKL2541,
iTcMc_DriveKL2531, iTcMc_DriveKL2541.
Reserved

TF5810

Version: 1.8.3

155

PLCopen Motion Control

4.3.29

ST_TcPlcMcLogBuffer

Available from version 3.0

A variable with this structure forms the LogBuffer of the library. Further information about creating a log
buffer can be found under FAQ #10 in the Knowledge Base [} 320].

The data in this structure must not be modified by the application.

Syntax
TYPE ST_TcMcLogBuffer:
STRUCT
    ReadIdx:        INT:=0;
    WriteIdx:       INT:=0;
    MessageArr:     ARRAY [0..19] OF ST_TcPlcMcLogEntry;
END_STRUCT
END_TYPE

Parameter

Name
ReadIdx
WriteIdx
MessageArr

Type
INT
INT
ARRAY

Description
The read index of the buffer.
The write index of the buffer.
The currently stored messages.

ST_TcPlcMcLogEntry [} 156]

4.3.30

ST_TcPlcMcLogEntry

Available from version 3.0

A variable with this structure contains a message of the LogBuffer of the library. Used as a component in
ST_TcPlcMcLogBuffer [} 156]. Further information about creating a log buffer can be found under FAQ #10 in
the Knowledge Base [} 320].

The data in this structure must not be modified by the application.

Syntax
TYPE ST_TcPlcMcLogEntry:
STRUCT
    TimeLow:    UDINT:=0;
    TimeHigh:   UDINT:=0;
    LogLevel:   DWORD:=0;
    Source:     DWORD:=0;
    Msg:        STRING(255);
    ArgType:    INT:=0;
    diArg:      DINT:=0;
    lrArg:      LREAL:=0;
    sArg:       STRING(255);
END_STRUCT
END_TYPE

156

Version: 1.8.3

TF5810

PLCopen Motion Control

Parameter

Name
TimeLow
TimeHigh
LogLevel

Type
UDINT
UDINT
DWORD

Source

DWORD

Msg
ArgType
diArg
lrArg
sArg

STRING
INT
DINT
LREAL
STRING

Description
The timestamp of the message. Records the time at which the message was
generated.

Indicates the urgency of the message. Only values from a specified pool of
numbers should appear here.
Indicates the source of the message. Only values from a specified pool of
numbers should appear here.
The message text with an optional placeholder for a variable component.
The type of the optional component.
If an optional component of type DINT is used, its value can be found here.
If an optional component of type LREAL is used, its value can be found here.
If an optional component of type STRING is used, its value can be found here.

4.3.31

ST_TcPlcRegDataItem

Available from version 3.0.7

This structure contains a parameter for a KL terminal. An ARRAY of elements of this type forms the type
ST_TcPlcRegDataTable [} 157].

Syntax
TYPE ST_TcPlcRegDataItem :
STRUCT
    Access:     INT:=0;
    Select:     INT:=-1;
    RegData:    WORD:=0;
END_STRUCT
END_TYPE

Parameter

Name
Access

Type
INT

Select
RegData

INT
WORD

Description
The type of the operation to be executed is coded here. Details can be found under
MC_AxUtiUpdateRegDriveTerm_BkPlcMc [} 295] or
MC_AxUtiUpdateRegEncTerm_BkPlcMc [} 297].
The address of the register in the terminal.
The parameters to be used for the operation to be executed.

4.3.32

ST_TcPlcRegDataTable

Available from version 3.0.7

This structure contains a parameter set for a KL terminal. Such a table is processed by the
MC_AxUtiUpdateRegDriveTerm_BkPlcMc [} 295] or MC_AxUtiUpdateRegEncTerm_BkPlcMc [} 297] function
blocks.

Syntax
TYPE ST_TcPlcRegDataTable :
STRUCT
    RegDataItem:    ARRAY [1..64] OF ST_TcPlcRegDataItem;
END_STRUCT
END_TYPE

TF5810

Version: 1.8.3

157

PLCopen Motion Control

Parameter

Name
RegDataItem

Type
ARRAY

Description

4.3.33

ST_TcHybridAxParam

Available from version 3.0.44

This structure contains additional parameters of the servo-electric/hydraulic axis. Also see about this:
Suitable procedures for axis commissioning [} 380].

The order of the parameters is not guaranteed.

Syntax
TYPE ST_TcHybridAxParam :
(* last modification: 20.02.2019 *)
STRUCT
    fPump_N_max:          LREAL;
    fPump_N_min:          LREAL;

    fPump_P_max:          LREAL;
    fPump_P_min:          LREAL;

    fPump_Q_fast_P:       LREAL;
    fPump_Q_slow_P:       LREAL;

    fPump_Q_fast_M:       LREAL;
    fPump_Q_slow_M:       LREAL;

    fPump_Q_leak:         LREAL;

    fPump_Enc_Offset:     LREAL;

    fCylinder_A_addP:     LREAL;
    fCylinder_A_addM:     LREAL;

    fRampTime:            LREAL;

    fAside_PrsScaling:    LREAL;
    fBside_PrsScaling:    LREAL;
    fSystem_PrsScaling:   LREAL;

    nPumpCavities:        DINT;
    nConcept:             DINT;

    nPump_EncType:        E_TcMcEncoderType:=iTcMc_EncoderSim;

    bRegenerative:        BOOL;
    bVirtual_A_addP:      BOOL;
    bVirtual_A_addM:      BOOL;
    bAside_PrsHiResADC:   BOOL;
    bBside_PrsHiResADC:   BOOL;
    bSystem_PrsHiResADC:  BOOL;

END_STRUCT
END_TYPE

158

Version: 1.8.3

TF5810

Parameter

PLCopen Motion Control

TF5810

Version: 1.8.3

159

PLCopen Motion Control

Name
fPump_N_max
fPump_N_min
fPump_P_max
fPump_P_min
fPump_Q_fast_P
fPump_Q_slow_P

fPump_Q_fast_M
fPump_Q_slow_M

fPump_Q_leak
fPump_Enc_Offset
fCylinder_A_addP

Type
LREAL
LREAL
LREAL
LREAL
LREAL
LREAL

LREAL
LREAL

LREAL
LREAL
LREAL

fCylinder_A_addM

LREAL

fRampTime

LREAL

fAside_PrsScaling
fBside_PrsScaling
fSystem_PrsScaling
nPumpCavities

LREAL
LREAL
LREAL
DINT

nConcept

DINT

nPump_EncType

E_TcMcEncoderType

bRegenerative

BOOL

bVirtual_A_addP

BOOL

bVirtual_A_addM

BOOL

Description
[rpm] The minimum and maximum permissible
pump speeds.

[bar] The minimum and maximum permissible
operating pressure of the pump.

[cm3/rev] The rotation-related flow rate of the pump
in rapid or force mode at the cylinder connection
acting in the positive direction.
[cm3/U] The rotation-related flow rate of the pump
in rapid or force mode at the cylinder connection
acting in the negative direction.
Reserved
Reserved
If, depending on the situation, an area effective for
oil demand is connected in the positive direction of
action, it must be identified here. This can also be
an oil demand required by an apparent area, which
actually bypasses the cylinder. In this case, the
area should be identified as "virtual".
If, depending on the situation, an area effective for
oil demand is connected in the negative direction of
action, it must be identified here. This can also be
an oil demand required by an apparent area, which
actually bypasses the cylinder. In this case, the
area should be identified as "virtual".
When switching between rapid and force mode, the
weighting factor for the velocity output and the
maximum attainable velocity are changed. A ramp
can be defined here, in order to avoid a
discontinuity.
The scaling pressures for the A-side, the B-side
and the system pressure detection are to be set
here.

Enter the number of pump chambers here. For
piston pumps the number of pistons must be set.
For internal gear pumps, the number of teeth on
the internal pinion must be set.
The circuit concept used for the servo-electric/
hydraulic axis must be specified here.

The encoder type [} 98] of the pump drive is
defined here. Only a small selection of encoder
types is available.

This is not the encoder on the cylinder.
This parameter indicates that the smaller cylinder
area is operated in oil exchange with the larger
cylinder area.
If an area that can be activated in the positive
direction of action is effective for the oil demand
but not for the force build-up, it must be identified
here.
If an area that can be activated in the negative
direction of action is effective for the oil demand
but not for the force build-up, it must be identified
here.

160

Version: 1.8.3

TF5810

Name
bAside_PrsHiResADC

Type
BOOL

bBside_PrsHiResADC

BOOL

bSystem_PrsHiResADC

BOOL

PLCopen Motion Control

Description
This parameter indicates that the pressure sensor
of the area with positive direction of action is read
with a 24-bit input terminal.
This parameter indicates that the pressure sensor
of the area with negative direction of action is read
with a 24-bit input terminal.
This parameter indicates that the pressure sensor
at the pressurized hydraulic reservoir is read with a
24-bit input terminal.

See Commissioning [} 380] for more information about axis commissioning.

4.3.34

ST_TcHybridAxRtData

Available from version 3.0.44

This structure contains additional runtime values of the servo-electric/hydraulic axis.

The parameter sequence is not guaranteed.

Syntax
TYPE ST_TcHybridAxRtData :
(* last modification: 05.12.2018 *)
STRUCT
    fPump_Angle:        LREAL;
    fPump_ModuloAngle:  LREAL;
    fPump_Speed:        LREAL;
    fPump_Torque:       LREAL;

    fMotor_N_max:       LREAL;
    fMotor_RefCurrent:  LREAL;
    fMotor_RefTorque:   LREAL;
    fMotor_PeekCurrent: LREAL;
    fMotor_PeekTorque:  LREAL;
    fMotor_NomCurrent:  LREAL;
    fMotor_NomTorque:   LREAL;

    fActive_Area_P:     LREAL;
    fActive_Area_M:     LREAL;
    fActive_Qmax_P:     LREAL;
    fActive_Qmax_M:     LREAL;

    fActive_Feed_P:     LREAL;
    fActive_Feed_M:     LREAL;

    fActive_N_max:      LREAL;

    fActive_Vmax_P:     LREAL;
    fActive_Vmax_M:     LREAL;

    fFeed_RampRate_P:   LREAL;
    fFeed_RampRate_M:   LREAL;
    fRamping_Feed_P:    LREAL;
    fRamping_Feed_M:    LREAL;

    bPump_Switched:     BOOL;
    bPump_AreaSwitched: BOOL;
    bMotor_EnablePwrMon:BOOL;
    bReRamp_FeedFactor: BOOL;
    bHydActualCall:     BOOL;
END_STRUCT
END_TYPE

TF5810

Version: 1.8.3

161

PLCopen Motion Control

Parameter

162

Version: 1.8.3

TF5810

Name
fPump_Angle

fPump_ModuloAngle

fPump_Speed

fPump_Torque

fMotor_N_max
fMotor_RefCurrent
fMotor_RefTorque
fMotor_PeekCurrent
fMotor_PeekTorque
fMotor_NomCurrent
fMotor_NomTorque
fActive_Area_P

fActive_Area_M

fActive_Qmax_P

fActive_Qmax_M

fActive_Feed_P

fActive_Feed_M

fActive_N_max

fActive_Vmax_P

fActive_Vmax_M

fFeed_RampRate_P

Type
LREAL

LREAL

LREAL

LREAL

LREAL
LREAL
LREAL
LREAL
LREAL
LREAL
LREAL
LREAL

LREAL

LREAL

LREAL

LREAL

LREAL

LREAL

LREAL

LREAL

LREAL

fFeed_RampRate_M

LREAL

fRamping_Feed_P

fRamping_Feed_M

bPump_Switched

bPump_AreaSwitched

bMotor_EnablePwrMon

LREAL

LREAL

BOOL

BOOL

BOOL

bReRamp_FeedFactor

BOOL

PLCopen Motion Control

Description
[°] The rotation-related actual angle of the motor and thus
also of the pump in the range 0 ... 360°.
[°] The cavity-related actual angle of the motor and thus also
of the pump in the range 0 ... 360°/number_of_cavities.
[°/s] The angular speed of the motor and thus also of the
pump. This value corresponds to six times the speed in rpm.
[%] The torque called up by the pump from the motor in
relation to its nominal torque.
[rpm] The maximum speed of the motor.
[A] The current reference value of the motor.
[Nm] The torque reference value of the motor.
[A] The current peak value of the motor.
[Nm] The torque peak value of the motor.
[A] The current rating of the motor.
[Nm] The torque rating of the motor.
[mm2] The currently effective area on the P-side of the
cylinder.
[mm2] The currently effective area on the M-side of the
cylinder.
[cm3/rev] The currently available quantity on the P-side of
the cylinder.
[cm3/rev] The currently available quantity on the M-side of
the cylinder.
[mm/rev] The currently available rotational feed rate on the
P-side of the cylinder.
[mm/rev] The currently available rotational feed rate on the
M-side of the cylinder.
[rpm] The currently available maximum speed of motor and
pump.
[mm/s] The currently available maximum velocity in positive
direction.
[mm/s] The currently available maximum velocity in negative
direction.
[mm/rev/cycle] The cycle-related feed factor change of a
current or already executed ramp in the positive direction of
cylinder movement.
[mm/rev/cycle] The cycle-related feed factor change of a
current or already executed ramp in the negative direction of
cylinder movement.
[mm/rev] The current ramped feed factor in positive direction
of cylinder movement.
[mm/rev] The current ramped feed factor in negative
direction of cylinder movement.
This signal indicates active switching of the pump to force
mode.
This signal indicates active activation of the areas for force
mode.
This signal indicates that the current and torque parameters
of the drive have been read and an accurate torque
calculation is available.
This signal starts the ramp for switching between rapid and
force mode.

TF5810

Version: 1.8.3

163

PLCopen Motion Control

Name
bHydActualCall

Type
BOOL

Description
This signal indicates that an instance of the function block
MC_AxRtHybridAxisActuals_BkPlcMc () has been called for
the servo-electric/hydraulic axis. Otherwise it is not ensured
that the actual values of the axis are fully determined and
the effects of pump or area switching are taken into account.
In this case, the axis is set to the error state and a message
is written to the log.

4.3.35

ST_TcPlcInputAnalog

Available from version 3.0.44

This structure contains variables for the evaluation of analog inputs.

Syntax
TYPE ST_TcPlcInputAnalog :
(* last modification: 20.02.2019 *)
STRUCT
    nADC:     DINT;
    nOpState: INT;
    bWcState: BOOL;
END_STRUCT
END_TYPE

Parameter

Name
nADC

Type
DINT

Description
nADC: The actual value is displayed here.

If this value is determined with a 16-bit terminal, it must be adapted. If it is a
signed value (e.g. from a ±10 V terminal), it must be assigned with a type
conversion INT_TO_DINT(). This automatically extends the sign to the upper
16 bits with the correct type. Otherwise negative values are interpreted as
very large positive values. If only positive values occur, this can be omitted.
In this case, direct mapping from 16 to 2 bit can be used, since the upper 16
bits remain unaffected.
nOpState: This signal indicates the operating state of the terminal.
bWcState: This signal indicates a problem with continuous data exchange with the
terminal.

nOpState
bWcState

INT
BOOL

4.3.36

ST_TcPctrlParam

This structure contains additional parameters that can be used for a force or pressure controller. The supply
of such a function block must be handled by the application.

The order of the parameters is not guaranteed

Syntax
TYPE ST_TcPctrlParam :
(* last modification: 30.07.2019 *)
STRUCT
    fkP:          LREAL;
    fTn:         LREAL;
    fTv:         LREAL;
    fPreset:     LREAL;
    fWuLimit:    LREAL;

164

Version: 1.8.3

TF5810

    nNf:         INT;

    bAlignAreas: BOOL;
END_STRUCT
END_TYPE

PLCopen Motion Control

Parameter

Name
fkP

fTn

fTv

fPreset
fWuLimit
nNf

Type
LREAL

Description
The proportional gain of the controller.

LREAL

LREAL

LREAL
LREAL
INT

The integration time constant of the controller. If it is set to 0.0, the I part is
switched off.
The rate time constant of the controller. If it is set to 0.0, the D part is switched
off.
This value initializes the I component when it is activated.
Limit for the I part.
The response of the D part usually generates an uneven signal that makes an
axis unstable. This parameter can be used to enable a moving average filter that
averages up to 100 values.
If this parameter is TRUE, the output of the controller is adjusted to the ratio of
the active areas of a cylinder depending on the direction. This can contribute to a
more stable control if the axis has to provide control in both directions.

bAlignAreas

BOOL

nNf: Undesired vibration

Strong filtering produces a phase error that can lead to vibration.

NOTICE

4.3.37

MC_Ref_Signal_Ref_BkPlcMc

A variable of this type is transferred to a MC_StepAbsoluteSwitch_BkPlcMc [} 308] or
MC_StepAbsoluteSwitchDetection_BkPlcMc [} 310] function block.

Syntax
TYPE MC_Ref_Signal_Ref_BkPlcMc:
STRUCT
    SignalSource:     E_SignalSource_BkPlcMc := E_SignalSource_BkPlcMc.SignalSource_Default;
    Level:            BOOL;
END_STRUCT
END_TYPE

TYPE E_SignalSource_BkPlcMc:
    SignalSource_Default := 0;
    (**)
END_TYPE

Parameter

Name
SignalSource

Type
E_SignalSource_BkPlcMc

Level

BOOL

Description
SignalSource: Selection of the signal source by
E_SignalSource_BkPlcMc.
Level:Input signal of the referencing cam.

4.3.38

E_TcMcJogMode

The constants in this listing are used to switch between different jog modes.

TF5810

Version: 1.8.3

165

PLCopen Motion Control

Syntax
TYPE E_TcMcJogMode :
(
MC_JOGMODE_STANDARD_SLOW, (* motion with standard jog parameters for slow motion *)
MC_JOGMODE_STANDARD_FAST, (* motion with standard jog parameters for fast motion *)
MC_JOGMODE_CONTINOUS, (* axis moves as long as the jog button is pressed using parameterized
dynamics *)
MC_JOGMODE_INCHING, (* axis moves for a certain relative distance *)
MC_JOGMODE_INCHING_MODULO (* axis moves for a certain relative distance - stop position is rounded
to the distance value *));
END_TYPE

Parameter

Name
MC_JOGMODE_STANDARD_SLOW

MC_JOGMODE_STANDARD_FAST

MC_JOGMODE_CONTINOUS

MC_JOGMODE_INCHING

MC_JOGMODE_INCHING_MODULO

4.4

System

4.4.1

Controller

Description
The axis moves as long as the signal at one of the jog inputs
is TRUE. The low velocity for manual functions specified in
AXIS_REF_BkPlcMc and standard dynamics are used. In
this operation mode the position, velocity and dynamics data
specified in the function block have no effect.
The axis moves as long as the signal at one of the jog inputs
is TRUE. The high velocity for manual functions specified in
AXIS_REF_BkPlcMc and standard dynamics are used. In
this operation mode the position, velocity and dynamics data
specified in the function block have no effect.
The axis moves as long as the signal at one of the jog inputs
is TRUE. The velocity and dynamics data specified by the
user are used. The position has no effect.
With rising edge at one of the jog inputs the axis is moved by
a certain distance which is specified via the position input.
The axis stops automatically, irrespective of the state of the
jog inputs. A new movement step is only executed once a
further rising edge is encountered. With each start the
velocity and dynamics data specified by the user are used.
Reserved

4.4.1.1

MC_AxCtrlAutoZero_BkPlcMc

Available from version 3.0

The function block executes an automatic zero compensation. This function block may only be used for zero
overlap valves.

166

Version: 1.8.3

TF5810

MC_AxUtiOffsetLatch_BkPlcMcExecute  BOOLOffsetLimit  LREAL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  DoneBOOL  ErrorUDINT  ErrorIDBOOL  LatchedLREAL  OffsetPLCopen Motion Control

 Inputs
VAR_INPUT
    Enable:         BOOL:=FALSE;
    EnableOnMoving: BOOL:=FALSE;
    OffsetLimit:    LREAL:=0.0;
    Tn:             LREAL:=0.0;
    Threshold:      LREAL:=0.1;
    Filter:         LREAL:=0.1;
END_VAR

Name
Enable
EnableOnMoving
OffsetLimit
Tn

Threshold
Filter

Type
BOOL
BOOL
LREAL
LREAL

LREAL
LREAL

Description
This input controls the activity of the compensation.
This input controls the activity of the compensation.
[V] The value in fZeroCompensation is limited to this value.
[s] The integral action time of the compensation. This is the time for a
change by 10 V. Values greater than 100 s are recommended.
[V] Parameter for the Done signal.
[s] Parameter for the Done signal.

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Error:          BOOL;
    ErrorID:        UDINT;
    Active:         BOOL;
    Limiting:       BOOL;
    Done:           BOOL;
END_VAR

Name
Error
ErrorID
Active

Type
BOOL
UDINT
BOOL

Description
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.
Indicates that the function block actively adjusts the value of fZeroCompensation in
ST_TcHydAxParam [} 130].

Limiting

BOOL

Done

BOOL

Indicates that the value of fZeroCompensation in ST_TcHydAxParam [} 130] has
reached the limit specified by OffsetLimit.
Indicates leveling out of the offset compensation.

Purpose of the function block

If a hydraulic cylinder is drifting when the position controller is switched off (kP=0.0), or if there is a
permanent lag error relative to the target when the position controller is active, this can be corrected by using
a zero overlap valve with offset compensation.

A hydraulic cylinder stops when it is in equilibrium of forces. In the simplest case (cylinder with identical
surfaces, no external forces by gravity or a process) this equilibrium is fulfilled, if the same pressure acts on
both surfaces. For a differential cylinder, the pressures must be proportional to the inverse of the surfaces.
Any external forces must be included. In order to achieve the required pressure conditions, a proportion of
the system pressure is required as pressure difference. In the case of a zero overlap valve, this is defined by
the pressure gain characteristic.

TF5810

Version: 1.8.3

167

PLCopen Motion Control

Another possible reason for an offset is a difference between the hydraulic zero point of the valve and the
logical zero point of the output hardware. These are unavoidable manufacturing tolerances.

Therefore, a small valve excitation with up to ±0.5 V is required. Refer to the data sheets provided by the
valve and hardware manufacturers for further information.

Behavior of the function block: Enable logic

As long as Enable for the function block or the axis controller is FALSE, the function block does not become
Active. The comparison value for monitoring the compensation is initialized and the time measurement for
the Done message is reset.

If the enable conditions are met and the axis is not in idle state (i.e. it is in motion), the time measurement for
the Done message is also reset.

If the enable conditions are met and the axis is in idle state, the function block 'Compensation&Timing' is
processed.

Irrespective of these preconditions, the function block' Feedback' is processed.

Enable logic:

168

Version: 1.8.3

TF5810

PLCopen Motion Control

Behavior of the function block: Compensation&Timing

A correction value is formed from the lag error and the response of the controller. The bandwidth of the
possible axis controller parameterization is taken into account. A delta value (maximum change in zero
compensation per cycle) is formed from this correction value and Tn. Tn defines a ramp time for an
increase by 10 V. The delta value is limited such that this ramp slope is not exceeded. In this way an
excessively fast change, during which the correction would become unstable, can be avoided. Values
greater than 100 seconds are recommended.

A tolerance threshold is used for compensation. In this case LagAmpDx (threshold value of the I component
in the position controller) is used.
If the correction value is greater than or equal to the tolerance threshold and the actual velocity is greater
than or equal to zero (i.e. the remaining correction value is not already reduced), the Active function block is
used and the compensation is reduced in each cycle by the delta value described.
If the correction value is less than or equal to the tolerance threshold and the actual velocity is less than or
equal to zero (i.e. the remaining correction value is not already reduced), the Active function block is used
and the compensation is reduced in each cycle by the delta value described.
If the magnitude of the correction value is smaller than the tolerance threshold, Active becomes FALSE.

If the compensation differs by more than the Threshold from the OldValue comparison value, the time
measurement is reset and the current compensation is updated as a new comparison value. Otherwise, the
time measurement is increased with the cycle time. In this way, the time required to accumulate a change in
compensation by at least the Threshold is logged.

Compensation&Timing:

TF5810

Version: 1.8.3

169

PLCopen Motion Control

Behavior of the function block: Feedback

The compensation is limited to ±OffsetLimit and signaled to Limiting.

Done is reported when the function block is active and the time measurement reaches the time set in Filter.
Example: If Threshold is set to 0.05 and Filter to 2.0, Done is reported if the compensation has been
readjusted by less than 0.05 V within the last 2 seconds.

170

Version: 1.8.3

TF5810

Feedback

PLCopen Motion Control

The limitation to the range specified by OffsetLimit applies even if the function block is not active.
The Limiting output is updated.

The value OffsetLimit and ST_TcHydAxParam [} 130].fZeroCompensation are regarded as offset voltage.
The value 10.0 therefore corresponds to full scale control. In general, a value between 0.1 and 1.0 makes
sense for OffsetLimit, depending on the application.

Integration of the function block in the application

In the call sequence for the function blocks of an axis, an MC_AxCtrlAutoZero_BkPlcMc function block
should appear immediately before the MC_AxRtFinish_BkPlcMc [} 246]. If an MC_AxStandardBody_BkPlcMc
[} 253] function block is called instead of the individual function blocks, MC_AxCtrlAutoZero_BkPlcMc should
be called before this function block.

TF5810

Version: 1.8.3

171

PLCopen Motion Control

Dangerous axis movement

 WARNING

If situations occur during axis operation, in which the axis has a controller enable pending but does not
display its normal motion behavior, the MC_AxCtrlAutoZero_BkPlcMc function block must be disabled.
Possible causes for such a situation including function block startup with or without transition to pressure
control or reduction of or switch-off of the supply. If this is not taken into account, the value of
fZeroCompensation in ST_TcHydAxParam [} 130] may run in any direction until the specified limit is
reached. As soon as the axis is responsive again at a later stage, a dangerous motion may be unavoidable.
In this case the positioning behavior will be severely affected. If the function block is called without
EnableOnMoving, it may no longer be able to automatically correct the shifted offset. In this case the axis
will stop outside the target window and never report the motion as complete, or only after a long time.

In combination with an MC_AxStandardBody_BkPlcMc [} 253] function block, all responses of the
MC_AxCtrlAutoZero_BkPlcMc function block are delayed by one PLC cycle. Usually this is no problem. If
this offset does cause problems, the individual function blocks for encoder etc. should be used, and the
MC_AxCtrlAutoZero_BkPlcMc function block should be called immediately before the
MC_AxRtFinish_BkPlcMc [} 246] function block.

4.4.1.2

MC_AxCtrlPressure_BkPlcMc

Available from version 3.0

The function block controls the pressure applied to an axis such that a specified default value is established
and maintained in the actual value selected by ReadingMode.

In most cases the actual pressure can be logged with function blocks of type
MC_AxRtReadPressureSingle_BkPlcMc [} 222] or MC_AxRtReadPressureDiff_BkPlcMc [} 220].

 Inputs
VAR_INPUT
    Enable:     BOOL:=FALSE;
    Reset:      BOOL:=TRUE;
    FirstAuxParamIdx: INT:=0;
    kP:         LREAL:=0.0;
    Tn:         LREAL:=0.0;
    ReadingMode:E_TcMcPressureReadingMode:=iTcHydPressureReadingDefault;
    PreSet:     LREAL:=0.0;
    WindupLimit:LREAL:=0.0;
END_VAR

172

Version: 1.8.3

TF5810

MC_AxCtrlPressure_BkPlcMcEnable  BOOLReset  BOOLFirstAuxParamIdx  INTkP  LREALTn  LREALReadingMode  E_TcMcPressureReadingModePreSet  LREALWindupLimit  LREALAlignAreas  BOOL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ErrorUDINT  ErrorIDBOOL  InWindupPLCopen Motion Control

Description
TRUE at this input activates the controller.
TRUE at this input resets the controller. The memory of
the I part is cleared.

Here a range in the AXIS_REF_BkPlcMc
[} 86].ST_TcHydAxParam [} 130].fCustomerData can be
activated as parameter interface.
The gain factor of the P part.
The integral action time of the I part.
The actual value to be controlled can be specified here.
AXIS_REF_BkPlcMc [} 86].ST_TcHydAxRtData
[} 141].fActPressure is selected as default value.
Here you can specify a default value for calculating an
initial value for the I part of the controller. The I part is
preloaded with this value on activation.
Here you can specify a limit value for the I part. Such a
limitation prevents extreme behavior of the I part in
situations where the path does not respond to controller
outputs.

Name
Enable
Reset

Type
BOOL
BOOL

FirstAuxParamIdx

INT

kP
Tn
ReadingMode

LREAL
LREAL
E_TcMcPressureReadin
gMode

PreSet

LREAL

WindupLimit

LREAL

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Error:      BOOL;
    ErrorID:    UDINT;
    InWindup:   UDINT;
END_VAR

Name
Error
ErrorID
InWindup

Type
BOOL
UDINT
UDINT

Description
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.
This output becomes TRUE if the I part is limited by WindupLimit.

Behaviour of the function block:

The function block investigates the axis interface that has been passed to it every time it is called. TRUE at
Reset puts the function block in an idle state, irrespective of the other control signals. Both the P component
and the I component are then deleted. Enable can be used to specified whether the function block assumes
the active state.

The input ReadingMode determines which variable is assigned the parameter to be controlled in the
stAxRtData structure.

• iTcHydPressureReadingDefault, iTcHydPressureReadingActPressure: fActPressure is controlled.

• iTcHydPressureReadingActForce: fActForce is controlled.

• Any other value deactivates the controller.

TF5810

Version: 1.8.3

173

PLCopen Motion Control

The set value has to be specified in fSetPressure in the stAxRtData structure of the axis.

First, the function block determines whether it has to assume or quit the active state. To this end the Enable
signal is evaluated. A rising edge causes the I component to be initialized with PreSet. If the output value
matching ST_TcHydAxRtData [} 141].in fSetPressure is known, it can be utilized for reaching the
compensated state more quickly. A P component is then calculated with kP, an I component with Tn. The
sum of these controller components is output as control value in ST_TcHydAxRtData [} 141].fSetSpeed. Since
this controller assumes the function of a control value generator, it cancels ST_TcHydAxRtData
[} 141].fLagCtrlOutput. The MC_AxRtFinish_BkPlcMc [} 246] function block to be positioned after the
controller function block then considers the response automatically.

The transition to the inactive state results in deletion of the controller components.

Integration of the function block in the application

A function block of this type must be called after the actual value and actual pressure acquisition. It handles
the full control of the axis and replaces any function block for control value generation that may be present.

A program example [} 321] #15 is available.

If a function block for control value generation and an MC_AxCtrlPressure_BkPlcMc function block
are present, these function blocks should either be called alternatively, or the
MC_AxCtrlPressure_BkPlcMc function block must follow after the control value function block, so
that it overwrites the outputs of this function block. Not all generator types allow both options.

A value greater than 0 in FirstAuxParamIdx can be used to instruct the function block to use three
consecutive values in the fCustomerData of the parameter structure as Tn, kP and PreSet. If the
address of a suitable ARRAY[..] OF STRING() is entered in Axis.pStAxAuxLabels, the parameters
are automatically assigned a name.

Commissioning

The four parameters kP, Tn, PreSet and WindupLimit enable the controller to be adapted to a range of
different tasks.

Control oscillations

NOTICE

During commissioning the axis may be subjected to the full system pressure, or damped or undamped
vibrations in a wide frequency range may occur. Appropriate measures must be taken, if there is a risk for
the axis or its surroundings. In any case, measures should be taken to enable fast deactivation of the
controls.

Initially 0.0 should be entered for Tn and Preset and 1.0 for WindupLimit. The controller now operates as a
pure P controller. Once a function block has started up and the controller is activated (Enable:=TRUE,
Reset:=FALSE, SetPressure:=set value), the maximum applicable value for kP can be determined. Increase
the value step-by-step, until an oscillation tendency becomes apparent. Use repeated deactivation and
activation to check whether the controller is actually stable. In practice the value will be between around 0.1
and 0.5.

The next parameter to be set is Tn. Initially, a relatively large value should be specified, e.g. 0.5. The actual
pressure should now be regulated to the set value with large inertia, but fairly precisely. Now determine the
maximum possible setting through step-by-step reduction. Again, use repeated deactivation and activation to
check whether the controller is actually stable. If there is a tendency to damped oscillation during activation,
Tn is already set too low.

The setting of WindupLimit does not directly influence the behavior of the controller. Rather, this parameter
is used to influence the transition behavior. If the controller is able to build up the pressure immediately
because the axis does not have to travel, the value of WindupLimit should be chosen such that the I
component is not greater than three to four times the value that is required according to valve characteristics.
In this way the pressure regulation can be achieved significantly more quickly. If the axis still has some way

174

Version: 1.8.3

TF5810

PLCopen Motion Control

to travel, a low value for this parameter will determine the motion of the axis until the working position is
reached. If the parameter is chosen too low, the axis will move very slowly or even stop. On the other hand,
a value that is too large will cause the axis to reach the working position with a rather high velocity, resulting
in steep pressure increase. The resulting peak pressure can be significant.

NOTICE
If possible, activation of a pressure controller should be avoided, unless the axis is very close to its working
position.

The value for PreSet can be used for two procedures. If the pressure regulator should continue the control
value of another function block continuously, its control value can be specified for the calculation of PreSet.
In this way it is possible to reduce or avoid step changes in the control value during activation of the
controller.

If the control value to be generated by the controller is known, a value that is close to this value can be
specified as PreSet. In this way it is possible to reduce the time, which the I component requires to establish
the control value. Since the P component is also active, a value should be set that is higher than the exact
value.

The ultimate aim when setting these parameters is to find a set of values that is appropriated for the
task by making small changes and assessing the controller characteristics.

Example for the behavior of the controller, if the axis first has to travel some distance before it can build up
the required pressure.

TF5810

Version: 1.8.3

175

PLCopen Motion Control

Example for the controller behavior, if the axis is able to build up the required pressure immediately.

4.4.1.3

MC_AxCtrlPressureFF_BkPlcMc

The function block regulates the pressure acting on an axis in such a way that the desired default value
SetPoint is set up and maintained in Actual. Alternatively, forces can also be used as actual values and
setpoints.

In most cases the actual pressure can be logged with function blocks of type
MC_AxRtReadPressureSingle_BkPlcMc [} 222] or MC_AxRtReadPressureDiff_BkPlcMc [} 220]. Function blocks
of type MC_AxRtReadForceSingle_BkPlcMc [} 218] or MC_AxRtReadForceDiff_BkPlcMc [} 215] are suitable for
an actual force.

 Inputs
VAR_INPUT
    Enable:       BOOL:=FALSE;
    Enable_P:     BOOL:=TRUE;
    Enable_I:     BOOL:=TRUE;
    Enable_D:     BOOL:=TRUE;
    Reset:        BOOL:=TRUE;

176

Version: 1.8.3

TF5810

MC_AxCtrlPressureFF_BkPlcMcEnable  BOOLEnable_P  BOOLEnable_I  BOOLEnable_D  BOOLReset  BOOLEnable_WuL  BOOLEnable_OutL  BOOLSetpoint  LREALActual  LREALFeedVelocity  LREALFeedCharge  LREALpParam  Pointer To ST_TcPctrlParam↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ErrorUDINT  ErrorIDBOOL  InWindupLREAL  DeviationLREAL  ProportionalLREAL  IntegratorLREAL  DifferentialLREAL  ResponsePLCopen Motion Control

    Setpoint:     LREAL:=0.0;
    Actual:       LREAL:=0.0;
    FeedVelocity: LREAL:=0.0;
    FeedCharge:   LREAL:=0.0;
    pParam:       POINTER TO ST_TcPctrlParam;
END_VAR

Description
TRUE at this input activates the controller.
A TRUE at this input activates the proportional component of the
controller.
A TRUE at this input activates the integrator of the controller if
the proportional component is active.
A TRUE at this input activates the differential component of the
controller if the proportional component is active.
TRUE at this input resets the controller. The memory of the I part
is cleared.
The setpoint of the controller.
The actual value of the controller.
The default value for a lower-level pre-control.
An instantaneously effective and permanent change in the
integral component.
The address of a structure with the controller parameters. If zero
is transferred here, the controller uses the parameters in
stAxParams.stPctrl.

Name
Enable
Enable_P

Type
BOOL
BOOL

Enable_I

BOOL

Enable_D

BOOL

Reset

Setpoint
Actual
FeedVelocity
FeedCharge

pParam

BOOL

LREAL
LREAL
LREAL
LREAL

POINTER TO
ST_TcPctrlParam

 Inputs/outputs

VAR_IN_OUT
    Axis:         AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Error:        BOOL;
    ErrorID:      UDINT;
    InWindup:     UDINT;
    Deviation:    LREAL;
    Proportional: LREAL;
    Integrator:   LREAL;
    Differential: LREAL;
    Response:     LREAL;
END_VAR

Name
Error
ErrorID
InWindup
Deviation
Proportional
Integrator
Differential
Response

Type
BOOL
UDINT
UDINT
LREAL
LREAL
LREAL
LREAL
LREAL

Description
A TRUE here signals an error.
A numerically encoded error indication.
A TRUE here signals that the integrator is limited by the WindupLimit.
The current controller deviation.
The current proportional component.
The current integral component.
The current differential component.
The output of the controller.

TF5810

Version: 1.8.3

177

PLCopen Motion Control

The controller has a complete PID core, the individual components of which can be switched on and off via
Boolean inputs regardless of their parameters.

In addition, a pre-control input is available, which makes it easier to adapt the control against a moving
object to the velocity of the object. If necessary, the component of the integrator can be instantaneously
changed with the input FeedCharge.

4.4.1.4

MC_AxCtrlSlowDownOnPressure_BkPlcMc

Available from version 3.0

The function block decelerates an axis such that a certain default value is not exceeded in the actual value
selected through ReadingMode. The rules of substitutional pressure control apply.

In most cases the actual pressure can be logged with function blocks of type
MC_AxRtReadPressureSingle_BkPlcMc [} 222] or MC_AxRtReadPressureDiff_BkPlcMc [} 220].

178

Version: 1.8.3

TF5810

MC_AxCtrlSlowDownOnPressure_BkPlcMcEnableP  BOOLEnableM  BOOLReset  BOOLFirstAuxParamIdx  INTkP  LREALTn  LREALPreSet  LREALReadingMode  E_TcMcPressureReadingMode↔Axis  Reference To AXIS_REF_BkPlcMcLREAL  ResponseBOOL  ActiveBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

 Inputs
VAR_INPUT
    EnableP:          BOOL:=FALSE;
    EnableM:          BOOL:=FALSE;
    Reset:            BOOL:=TRUE;
    FirstAuxParamIdx: INT:=0.0;
    kP:               LREAL:=0.0;
    Tn:               LREAL:=0.0;
    PreSet:           LREAL:=0.0;
    ReadingMode:      E_TcMcPressureReadingMode:=iTcHydPressureReadingDefault;
END_VAR

E_TcMcPressureReadingMode [} 119]

Name
EnableP

EnableM

Reset

Type
BOOL

BOOL

BOOL

FirstAuxParamIdx

INT

kP
Tn
PreSet

LREAL
LREAL
LREAL

ReadingMode

E_TcMcPressureReadin
gMode

 Inputs/outputs

VAR_IN_OUT
    Axis:             AXIS_REF_BkPlcMc;
END_VAR

Description
TRUE at this input enables the controller to influence the
output value during a motion in positive direction.
TRUE at this input enables the controller to influence the
output value during a motion in negative direction.
TRUE at this input resets the controller. The memory of
the I part is cleared.

Here a range in the AXIS_REF_BkPlcMc
[} 86].ST_TcHydAxParam [} 130].fCustomerData can be
activated as parameter interface.
The gain factor of the P part.
The integral action time of the I part.
Here you can specify a default value for calculating an
initial value for the I part of the controller. The I part is
preloaded with this value on activation.
The actual value to be controlled can be specified here.
AXIS_REF_BkPlcMc [} 86].ST_TcHydAxRtData
[} 141].fActPressure is selected as default value.

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Response:         LREAL;
    Active:           BOOL;
    Error:            BOOL;
    ErrorID:          UDINT;
END_VAR

Name
Response
Active

Error
ErrorID

Type
LREAL
BOOL

BOOL
UDINT

Description
The output value of a pressure controller.
TRUE at this output indicates that the function block generates a response in
order to take over the pressure control.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

TF5810

Version: 1.8.3

179

PLCopen Motion Control

Behaviour of the function block:

TRUE at Reset puts the function block in an idle state, irrespective of the other control signals. Active is
then FALSE and Response := 0.0, since both the P component and the I component are deleted.

The input ReadingMode determines which variable is assigned the parameter to be controlled in the
stAxRtData structure.

• iTcHydPressureReadingDefault, iTcHydPressureReadingActPressure: fActPressure is controlled.

• iTcHydPressureReadingActForce: fActForce is controlled.

• Any other value deactivates the controller.

The set value has to be specified in fSetPressure in the stAxRtData structure of the axis.

During active operation the behavior of the function block is determined by the inputs EnableP and
EnableM. They determine whether the function block should intervene in positive or negative direction
during a motion. Note that the function block is tasked to counteract an active travelling motion. EnableP
should therefore be set if travelling motion in positive direction should not exceed a specified pressure. In
opposite direction of travel EnableM enables a pressure-limiting controller response in positive direction.

First, the function block determines whether it has to assume or quit the active state. To this end the signals
EnableP, EnableM, the sign of ST_TcHydAxRtData [} 141].fSetSpeed and the difference between
SetPressure and the selected actual value are evaluated.

During transition to the active state the I component is initialized with PreSet. It is loaded with a starting
value, which in combination with ST_TcHydAxRtData [} 141].fSetSpeed results in the value of PreSet. If the
output value matching fSetPressure is known, it can be utilized for reaching the compensated state more
quickly. In practice, the choice of this parameter should be made dependent on the behavior of the controlled
system. This is mainly influenced by the flexibility of the pressed in object, but also by the selected velocity. If
the increase is rather slow compared with the Tn used, the current control value from ST_TcHydAxRtData
[} 141].fSetSpeed should be used as preset value. If the actual pressure responds with a rapid increase, it is
advisable to use a value, which takes into account the set pressure and the pressure amplification of the
valve.

A P component is then calculated with kP, an I component with Tn. The sum of these controller components
is output as Response, and the state of the controller is indicated as TRUE at Active.

The transition to the inactive state results in deletion of the controller components and is indicated with
FALSE at Active.

Integration of the function block in the application

A function block of this type must be called after the actual value and actual pressure acquisition, and after
the control value generation. If function blocks are called for velocity or position control, these must also be
positioned before the pressure regulator function block, or the responses of the controllers should be
coordinated with due diligence.

Although the pressure regulator calculates a response, it is not entered in the ST_TcHydAxRtData [} 141]
structure. This is done by the application, depending on Active and taking into account signals of other
controllers. Usually, Response is assigned to the variable ST_TcHydAxRtData [} 141].fLagCtrlOutput. The
MC_AxRtFinish_BkPlcMc [} 246] function block to be positioned after the controller function block then
considers the response automatically.

A value greater than 0 in FirstAuxParamIdx can be used to instruct the function block to use three
consecutive values in the fCustomerData of the parameter structure as Tn, kP and PreSet. If the
address of a suitable ARRAY[..] OF STRING() is entered in Axis.pStAxAuxLabels, the parameters
are automatically assigned a name.

180

Version: 1.8.3

TF5810

4.4.1.5

MC_AxCtrlSlowDownOnPressureEx_BkPlcMc

PLCopen Motion Control

The function block brakes an axis in such a way that the actual value in Actual does not exceed the setpoint
specified in Setpoint.

In most cases, the actual pressure or actual force can be measured with function blocks of the type
MC_AxRtReadPressureSingle_BkPlcMc [} 222] or MC_AxRtReadPressureDiff_BkPlcMc [} 220] or
MC_AxRtReadForceSingle_BkPlcMc [} 218] or MC_AxRtReadForceDiff_BkPlcMc [} 215].

 Inputs
VAR_INPUT
    Enable:           BOOL:=FALSE;
    EnableRelief:     BOOL:=FALSE;
    Setpoint:         LREAL;
    Actual:           LREAL;
    FeedVelocity:     LREAL:=0.0;
    pParam:           POINTER TO ST_TcPctrlParam:=0;
END_VAR

Name
Enable
EnableRelief

Setpoint
Actual
FeedVelocity

pParam

Type
BOOL
BOOL

LREAL
LREAL
LREAL

POINTER TO
ST_TcPctrlParam

Description
A TRUE at this input enables the controller.
A TRUE at this input allows the controller to actively back off if
necessary.
The setpoint for the actual value to be limited.
The current value of the variable to be limited.
If the object against which the pressure or force is applied is
moving, its velocity can be pre-controlled here.

The address of a structure of the type ST_TcPctrlParam [} 164]
can be transferred here. If this input is unused or if 0 is applied to
it, the control parameters from the parameters of the axis are
used.

 Inputs/outputs

VAR_IN_OUT
    Axis:             AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:             BOOL;
    Active:           BOOL;
    Error:            BOOL;
    ErrorID:          UDINT;
END_VAR

TF5810

Version: 1.8.3

181

MC_AxCtrlSlowDownOnPressureEx_BkPlcMcEnable  BOOLEnableRelief  BOOLSetpoint  LREALActual  LREALFeedVelocity  LREALpParam  Pointer To ST_TcPctrlParam↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  ActiveBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

Name
Busy

Type
BOOL

Active

BOOL

Error
ErrorID

BOOL
UDINT

Description
A TRUE at this output indicates that the function block is enabled. This does not
necessarily mean that it is actively intervening in the behavior of the axis.
A TRUE at this output indicates that the function block is enabled and is actively
intervening in the behavior of the axis.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

The function block is placed in an idling state by a FALSE at Enable or if the axis is not enabled by the
controller. Busy, Active and Error are then FALSE.

If Enable is TRUE, a series of prerequisites is checked.

• Setpoint must be recognizably different from 0.0.

• The axis must be distance-controlled.

• The axis must not be operated with an external setpoint generator.

• The product of Setpoint and kP of the controller must not fall below the creep velocity of the axis. In

this case, the axis would no longer be able to move sensibly.

If one of these prerequisites is not satisfied, then Busy and Active are FALSE and Error is TRUE.
Otherwise Busy is TRUE.

The sign of Setpoint determines the working direction of the function block. If it is positive, it intervenes
when Actual approaches the setpoint from below in an ascending direction. Active is only TRUE if the
function block intervenes.

The function block determines the difference between the setpoint and the actual value, taking into account
the working direction, and calculates the still permissible velocity with the kP from the parameter structure
used. If the target velocity exceeds this value, Active goes TRUE and the velocity is limited.

When the actual value reaches the setpoint, the still permissible velocity = 0.0 and the axis should come to a
standstill now at the latest. If the actual value continues to increase, an opposite movement is only triggered
with EnableRelief.

In some applications, a force or pressure must be exerted against a moving object. In this case, the control
accuracy can be improved by providing the controller with a suitable pre-control at FeedVelocity.

Integration of the function block in the application

A function block of this type must be called after the actual value and actual pressure acquisition, and after
the control value generation. If function blocks are called for velocity or position control, they must also be
placed in front of the pressure controller function block. The MC_AxRtFinish_BkPlcMc [} 246] function block to
be positioned after the controller function block then considers the response automatically.

4.4.1.6

MC_AxCtrlStepperDeStall_BkPlcMc

The function block monitors the motion of a stepper motor axis, which is operated with an encoder.

182

Version: 1.8.3

TF5810

MC_AxCtrlStepperDeStall_BkPlcMcEnableAcc  BOOLEnableDec  BOOLReset  BOOLUseKL2531State  BOOLResetRefOnError  BOOLFirstAuxParamIdx  INTVeloLimit  LREALLimitFilter  LREALUpdateFilter  LREAL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ActiveBOOL  ActivatedBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

It is essential to use a real encoder (not an encoder emulation based on pulse counting of an output
terminal) in order to ensure correct function of this function block.

The application of such a function block can result in stalling (torque discontinuity). It therefore
cannot be assumed that the velocity is constant.

 Inputs
VAR_INPUT
    EnableAcc:       BOOL:=FALSE;
    EnableDec:       BOOL:=FALSE;
    Reset:           BOOL:=FALSE;
    UseKL2531State:  BOOL:=FALSE;
    ResetRefOnError: BOOL:=FALSE;
    FirstAuxParamIdx:INT:=0;
    VeloLimit:       LREAL:=0.0;
    LimitFilter:     LREAL:=0.0;
    UpdateFilter:    LREAL:=0.0;
END_VAR

Name
EnableAcc
EnableDec
Reset
UseKL2531State

Type
BOOL
BOOL
BOOL
BOOL

ResetRefOnError

BOOL

FirstAuxParamIdx

INT

VeloLimit

LimitFilter

UpdateFilter

LREAL

LREAL

LREAL

Description
These inputs determine whether the monitoring may intervene during the
acceleration and braking phases.

This input controls the activity of the controller.
If TRUE is transferred here, the function block evaluates
ST_TcPlcDeviceInput [} 149].bTerminalState.
If TRUE is transferred here, the function block clears the reference flag
of the axis.

Here a range in the AXIS_REF_BkPlcMc [} 86].ST_TcHydAxParam
[} 130].fCustomerData can be activated as parameter interface.
The threshold for the velocity deviation, from which the stall situation is
detected.
The time over which an excessive velocity deviation must be present
continuously for the stall situation to be detected.
The time constant, with which the velocity control value in the function
block is adjusted to the actual velocity.

 Inputs/outputs

VAR_IN_OUT
    Axis:            AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Active:          BOOL;
    Activated:       BOOL;
    Error:           BOOL;
    ErrorID:         UDINT;
END_VAR

TF5810

Version: 1.8.3

183

PLCopen Motion Control

Name
Active
Activated

Error
ErrorID

Type
BOOL
BOOL

BOOL
UDINT

Description
Indicates that a stall situation was detected.
Indicates that a stall situation was detected since the last start of an active axis
movement.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behaviour of the function block:

During each call the function block checks whether it has to change the state. It goes in the active state if the
internal motion phase permits this under the rules of EnableAcc, EnableDec and the velocity error
continuously exceeds the value of VeloLimit for at least LimitFilter. EnableAcc enables the function block
to intervene during phases with constant phases or phases with rising magnitude. EnableDec enables the
activity of the function block for phases with falling magnitude or constant velocity. Active and Activated are
set during the transition to the active state.

The function block changes to inactive state if the velocity error was reduced to half the value of VeloLimit.
Active is cancelled during the transition to the inactive state.

In active state the control value is adjusted to the actual velocity with the time constant UpdateFilter. If the
time constant is set to 0.0, the actual velocity is applied directly.

In inactive state Activated is cancelled, if the axis leaves the idle state and starts an active motion.

Since the function block evaluates the difference between set and actual velocity, it is important to
set the reference velocity correctly when this function block is used. Imprecise setting of this
parameter can result in unnecessary intervention by the function block in the motion.

The following Scope View shows a positioning, during which an obstacle was encountered twice. In each
case the axis stopped completely.

184

Version: 1.8.3

TF5810

PLCopen Motion Control

Integration of the function block in the application

A function block of this type must be called after the actual value acquisition and control value generation.
The function block superimposes its response with that of the control value generator and enters it in the
ST_TcHydAxRtData [} 141]. The MC_AxRtFinish_BkPlcMc [} 246] function block to be positioned after the
controller function block then considers the response automatically.

A value greater than 0 in FirstAuxParamIdx can be used to instruct the function block to use three
consecutive values in the fCustomerData of the parameter structure as VeloLimit, LimitFilter and
UpdateFilter. If the address of a suitable ARRAY[..] OF STRING() is entered in
Axis.pStAxAuxLabels, the parameters are automatically assigned a name.

4.4.1.7

MC_AxRtPosPiControllerEx_BkPlcMc

Available from version 3.0.40

The function block can be used as an alternative to the default position controller. It is called after the
MC_AxRuntime_BkPlcMc() function block (setpoint generator and default position controller). This
arrangement overwrites the responses of the default position controller.

 Inputs
VAR_INPUT
    Reset:          BOOL:=FALSE;
    I_Enable:       BOOL:=FALSE;
END_VAR

Name
Reset
I_Enable

Type
BOOL
BOOL

Description
This input deletes all internal and external controller responses.
This input controls the activity of the I part.

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    SetPos:         LREAL;
    SetVelo:        LREAL;
    Response:       LREAL;
    InWindup:       BOOL;
END_VAR

TF5810

Version: 1.8.3

185

MC_AxRtPosPiControllerEx_BkPlcMcReset  BOOLI_Enable  BOOL↔Axis  Reference To AXIS_REF_BkPlcMcLREAL  SetPosLREAL  SetVeloLREAL  ResponseBOOL  InWindupPLCopen Motion Control

Name
SetPos
SetVelo
Response
InWindup

Type
LREAL
LREAL
LREAL
BOOL

Description
[mm] The set position that becomes effective at the internal controller.
[mm/s] The set velocity that becomes effective at the internal controller.
[mm/s] The controller response.
Here, the limitation of the I part that has become active is signaled.

Purpose of the function block

The default position controller integrated in the MC_AxRuntime_BkPlcMc() [} 237] function block cannot meet
the control requirements of some applications, due to its simple structure. The
MC_AxRtPosPiControllerEx_BkPlcMc() function block is available for such applications. It supports the
following control components:

• Position P controller

• Position I controller with threshold and Windup limit

• Position D controller (realized as velocity P controller) with attenuation time

• Condition feedback for the actual velocity

• Condition feedback for the actual acceleration

• Compensation of the static effect of the condition feedback for the actual velocity

Velocity pre-control is activated after the controller. The same applies to any activated linearizations.

The controller is enabled with V3.0.40. The extended parameters are supported by the
PlcMcManager released with this version.

Structure of the controller

The component marked with an asterisk * prepares the set value for the I component of the controller when
the setpoint generator is path-controlled. This is necessary because the set position provided by the setpoint
generator jumps to the target position when the braking distance is reached. With time-controlled setpoint
generator, the component is transparent.

186

Version: 1.8.3

TF5810

PLCopen Motion Control

Not shown here: TRUE on Reset, or a missing controller enable of the axis deletes both the I
component and the controller output.

The I component has a threshold value Dx, which prevents a response to small deviations. For technical
reasons, this parameter is limited to at least 2/3 incremental weighting of the encoder. If the I component is
to be inactive, set Ti to zero.

The implementation of the D component takes advantage of the fact that the differentiated set position is
provided by the setpoint generator. An actual velocity is determined by differentiating the actual position.
Under this condition, the differentiation time constant Td acts as a proportionality factor. If the D component
is to be inactive, set the time constant Td to zero.

Three branches are implemented in the condition feedback:

• Velocity activation: The actual velocity is filtered and activated with a weighting factor. As it is
subtracted, it has an attenuating effect. If the connection is to be inactive, set KCfb_V to zero.

• Acceleration activation: The actual velocity is differentiated, filtered and activated with a weighting

factor. As it is subtracted, it has an attenuating effect. If the connection is to be inactive, set KCfb_A to
zero.

• A velocity activation generates a statically effective reduction of the velocity pre-control. In the case of

path-controlled positioning, this generates a noticeable velocity deviation. With time-controlled
positioning, this effect is compensated, as far as possible, by the continuously active position control.
This undesirable side-effect of velocity feedback is eliminated by automatic adjustment of the pre-
control. Deactivating the velocity activation also deactivates this compensation.

Velocity pre-control is activated after the controller. The weighting is fixed at 1.0 when the setpoint generator
is path-controlled and cannot be reduced.

If linearization is activated, it takes place after the controller and is not shown here.

4.4.2

Drive

4.4.2.1

MC_AxRtDrive_BkPlcMc

Available from version 3.0

The function block performs preparation of the control value for the axis for it to be output on a hardware
module. To this end a function block is called depending on the value set as nDrive_Type in
Axis.ST_TcHydAxParam [} 130], which takes into account the special features of the hardware module.

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

TF5810

Version: 1.8.3

187

MC_AxRtDrive_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ErrorUDINT  ErrorIdPLCopen Motion Control

 Outputs

VAR_OUTPUT
    Error:      BOOL;
    ErrorID:    UDINT;
END_VAR

Name
Error
ErrorID

Type
BOOL
UDINT

Description
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behaviour of the function block:

The function block investigates the axis interface that has been passed to it every time it is called. A number
of problems can be detected and reported during this process:

• If nDrive_Type in pStAxParams is set to an unacceptable value, the function block reacts with Error

and ErrorID:=dwTcHydErrCdDriveType. If the pointer pStAxRtData for the axis has been initialized, it
is placed into a fault state.

• If one of the specific sub-function-blocks detects a problem, it will (if possible) place the axis into a fault

state. This error is then echoed at the outputs of the MC_AxRtDrive_BkPlcMc.

If it is possible to carry out these checks without encountering any problems, the control value for the axis is
processed appropriately for the nDrive_Type [} 94] in Axis.ST_TcHydAxParam [} 130].

Information about the necessary linking of I/O components with the input and output structures of the axis
may be found in the Knowledge Base [} 320] under FAQ #7.

If only the usual blocks (encoder, generator, finish, drive) for the axis are to be called, a block of type
MC_AxStandardBody_BkPlcMc [} 253] should be used for simplicity.

The function blocks MC_AxUtiReadRegDriveTerm_BkPlcMc [} 293] and
MC_AxUtiWriteRegDriveTerm_BkPlcMc [} 302] are available for asynchronous data exchange with I/O
devices of the KL series.

iTcMc_DriveAx2000_B110A

The function block handles the evaluation of the actual values of an AX2000 servo actuator at the EtherCAT
fieldbus. This assumes that the connected motor is equipped with an absolute encoder. If a motor is
operated with a resolver, iTcMc_DriveAx2000_B110R should be set.

During manual insertion or automatic detection of a drive actuator the TwinCAT System Manager
will suggest to insert an NC axis in the project and connect it with this actuator. If this actuator is to
be controlled with the hydraulic system library, it is essential to decline this suggestion.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions overlap with those of the encoder function block and
described there. See also iTcMc_EncoderAX2000_B110A [} 199].

iTcMc_DriveAx2000_B110R

The function block handles the processing of the axis control value for output on an AX2000 servo drive at
the EtherCAT fieldbus.

During manual insertion or automatic detection of a drive actuator the TwinCAT System Manager
will suggest to insert an NC axis in the project and connect it with this actuator. If this actuator is to
be controlled with the hydraulic system library, it is essential to decline this suggestion.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions overlap with those of the encoder function block and
described there. See also iTcMc_EncoderAx2000_B110R [} 200].

188

Version: 1.8.3

TF5810

PLCopen Motion Control

iTcMc_DriveAx2000_B200R, iTcMc_DriveAx2000_B900R

The function block handles the processing of the axis control value for output on an AX2000 servo drive at
the Beckhoff Lightbus (B200) or RtEthernet fieldbus (B900).

During manual insertion or automatic detection of a drive actuator the TwinCAT System Manager
will suggest to insert an NC axis in the project and connect it with this actuator. If this actuator is to
be controlled with the hydraulic system library, it is essential to decline this suggestion.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions overlap with those of the encoder function block and
described there. See also iTcMc_EncoderAx2000_B200R [} 201].

iTcMc_DriveAx2000_B750A

The function block handles (from V3.0.26) processing of the control value of the axis for output at an AX2000
servo actuator at the Sercos fieldbus. The function block handles the evaluation of the actual values of an
AX2000 servo actuator at the EtherCAT fieldbus.

During manual insertion or automatic detection of a drive actuator the TwinCAT System Manager
will suggest to insert an NC axis in the project and connect it with this actuator. If this actuator is to
be controlled with the hydraulic system library, it is essential to decline this suggestion.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions overlap with those of the encoder function block and
described there. See also iTcMc_EncoderAX2000_B750A [} 202].

Note a number of special features. Further information can be found in the Knowledge Base.

iTcMc_DriveAx5000_B110A, iTcMc_DriveAx5000_B110SR

The function block handles the processing of the axis control value for output on an AX5000 servo actuator
at the EtherCAT fieldbus. The function block handles the evaluation of the actual values of an AX2000 servo
actuator at the EtherCAT fieldbus. If motor is operated with a resolver, iTcMc_EncoderAx5000_B110SR
should be set.

During manual insertion or automatic detection of a drive actuator the TwinCAT System Manager
will suggest to insert an NC axis in the project and connect it with this actuator. If this actuator is to
be controlled with the hydraulic system library, it is essential to decline this suggestion.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions overlap with those of the encoder function block and
described there. See also iTcMc_EncoderAX5000_B110A [} 202].

A list of successfully tested compatible devices can be found under iTcMc_EncoderAX5000_B110A [} 202].

Note a number of special characteristics. Further information can be found in the Knowledge Base.

iTcMc_DriveCoE_DS402

The function block handles the evaluation of the actual values of a servo actuator with CoE DS402 profile at
the EtherCAT fieldbus.

During manual insertion or automatic detection of a drive actuator the TwinCAT System Manager
will suggest to insert an NC axis in the project and connect it with this actuator. If this actuator is to
be controlled with the hydraulic system library, it is essential to decline this suggestion.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions overlap with those of the encoder function block and
described there. See also iTcMc_EncoderCoE_DS402A [} 204] and iTcMc_EncoderCoE_DS402SR [} 205].

A list of successfully tested compatible devices can be found under iTcMc_EncoderCoE_DS402SR [} 205].

TF5810

Version: 1.8.3

189

PLCopen Motion Control

Currently only drives with resolver or single-turn encoders are supported.

iTcMc_Drive_CoE_DS408

The function block handles the processing of the axis control value for output to a proportional valve at the
EtherCAT fieldbus. The valve must support the CiA DS408 profile.

I/O variable
see note
see note
see note
WcState

InfoData.State

InfoData.AdsAddr

Use
Output of the velocity signal.

Interface.Variable
ST_TcPlcDeviceInput.nDacOut
ST_TcPlcDeviceOutput.uiDriveCtrl Device control.
ST_TcPlcDeviceInput.uiStatus
ST_TcPlcDeviceInput.wDriveWcSt
ate
ST_TcPlcDeviceInput.uiDriveBoxSt
ate
ST_TcPlcDeviceInput.sDrvAdsAdd
r

Device status
Connection monitoring.

Monitoring of online status

Automatic identification.

The names of the process data exchanged with the device are specified via the XML file of the
manufacturer.

The valve must support the following Index.SubIndex combinations.

Index
1000
1008
1018
1018

Subindex
0
0
1
2

Meaning
Identification
Device name (optional)
Manufacturer ID
Device type

The following list of compatible devices is naturally incomplete. It is not a recommendation but is merely
intended for information. Beckhoff cannot guarantee trouble-free operation of the listed devices. If a
manufacturer or one of their devices is not listed, trouble-free operation may well be possible, but is not
guaranteed.

Manufacturer
Moog
Parker

Type
D638Exxx
DxxFP /DxxFE /TDP /TPQ

Description
Proportional valve
Proportional valve

x: Represents a placeholder for different characters.

iTcMc_DriveIx2512_1Coil

The function block deals with processing of the axis control value for output on an IP2512 PWM fieldbus
module.

I/O variable
Data out

Interface.Variable
ST_TcPlcDeviceOutput.uiDacOutA Output of the PWM factor.

Use

iTcMc_DriveIx2512_2Coil

The function block deals with processing of the axis control value for output on an IP2512 PWM fieldbus
module.

190

Version: 1.8.3

TF5810

PLCopen Motion Control

Interface.Variable
ST_TcPlcDeviceOutput.uiDacOutA Output of the PWM factor for coil 1.
ST_TcPlcDeviceOutput.uiDacOutB Output of the PWM factor for coil 2.

Use

I/O variable
Data out
Data out

iTcMc_DriveEL2535

The function block prepares the control value of the axis for output on a current-controlled PWM output
terminal. This terminal provides two independent output stages and can be used for the following valve
types:

Proportional valve with spring center position and two coils without permanent magnets:
nDrive_Type = iTcMc_DriveEL2535_2Coil.

Both channels are required for one valve. The terminal cannot be used for another valve at the same time.

With this type of valve, a proportion of the full current in the directionally active coil with currentless
countercoil is required to move the slider to the desired position. For -100% .. 0% .. +100% control, the
terminal block generates the output values 0 .. 0 .. 32767 in uiDacOutA and 32767 .. 0 .. 0 in uiDacOutB.

I/O variable
Channel1.PWM Output
Channel2.PWM Output

Channel1.Status
Channel2.Status

Channel1.Control
Channel2.Control

InfoData.State

WcState

InfoData.AdsAddr

Use

DO NOT USE!
Status of the first device channel
Status of the second device
channel

Interface.Variable
ST_TcPlcDeviceOutput.uiDacOutA Output of the PWM factor for coil 1.
ST_TcPlcDeviceOutput.uiDacOutB Output of the PWM factor for coil 2.
ST_TcPlcDeviceOutput.nDacOut
ST_TcPlcDeviceInput.uiStatus
ST_TcPlcDeviceInput.uiTerminalSt
ate2
ST_TcPlcDeviceOutput.uiDriveCtrl Control of the first device channel
ST_TcPlcDeviceOutput.uiTerminal
Ctrl2
ST_TcPlcDeviceInput.uiDriveBoxSt
ate
ST_TcPlcDeviceInput.wDriveWcSt
ate
ST_TcPlcDeviceInput.sDrvAdsAdd
r

Control of the second device
channel
Connection monitoring, condition
monitoring.
Connection monitoring.

Automatic identification,
parameterization.

Proportional valve with spring end position and coil without permanent magnets:
nDrive_Type = iTcMc_DriveEL2535_1Coil.

Only one channel is required here. The terminal can also be used for another valve. The I/O variables of the
second channel must be used for this purpose.

With this type of valve, 50% of the full power supply is required to move the slider to the center position. The
terminal module generates the output values 0 .. 16384 .. 32767 for -100% .. 0% .. +100% control.

TF5810

Version: 1.8.3

191

PLCopen Motion Control

I/O variable

Channel1.PWM Output
Channel1.Status

Channel1.Control
InfoData.State

WcState

InfoData.AdsAddr

Use

Interface.Variable
ST_TcPlcDeviceOutput.uiDacOutA DO NOT USE!
ST_TcPlcDeviceOutput.uiDacOutB DO NOT USE!
ST_TcPlcDeviceOutput.nDacOut Output of the PWM factor.
ST_TcPlcDeviceInput.uiStatus

Device status

ST_TcPlcDeviceOutput.uiDriveCtrl Device control.
ST_TcPlcDeviceInput.uiDriveBoxSt
ate
ST_TcPlcDeviceInput.wDriveWcSt
ate
ST_TcPlcDeviceInput.sDrvAdsAdd
r

Automatic identification,
parameterization.

Connection monitoring, condition
monitoring.
Connection monitoring.

Proportional valve with spring center position and a coil with permanent magnets:
nDrive_Type = iTcMc_DriveEL2535_1Coil.

Only one channel is required here. The terminal can also be used for another valve. The I/O variables of the
second channel must be used for this purpose.

This type of valve requires a bipolar current supply, which corresponds to the operating principle of a ±10 V
terminal. The output value generated by the terminal block is to be adjusted as follows AFTER the drive
function block has been called by the application:

ST_TcPlcDeviceOutput.nDacOut := 2 * (ST_TcPlcDeviceOutput.nDacOut - 16384);

I/O variable

Channel1.PWM Output
Channel1.Status
Channel1.Control
InfoData.State

WcState

InfoData.AdsAddr

iTcMc_DriveEL4132

Use

Interface.Variable
ST_TcPlcDeviceOutput.uiDacOutA DO NOT USE!
ST_TcPlcDeviceOutput.uiDacOutB DO NOT USE!
ST_TcPlcDeviceOutput.nDacOut Output of the PWM factor.
ST_TcPlcDeviceInput.uiStatus
ST_TcPlcDeviceOutput.uiDriveCtrl Device control.
ST_TcPlcDeviceInput.uiDriveBoxSt
ate
ST_TcPlcDeviceInput.wDriveWcSt
ate
ST_TcPlcDeviceInput.sDrvAdsAdd
r

Automatic identification,
parameterization.

Device status

Connection monitoring, condition
monitoring.
Connection monitoring.

The function block deals with processing of the axis control value for output on a ±10 V output terminal.

I/O variable
Output

InfoData.State

WcState

iTcMc_DriveEL7031

Interface.Variable
ST_TcPlcDeviceOutput.nDacOut Operation: Output of the velocity

Use

ST_TcPlcDeviceInput.uiDriveBoxSt
ate
ST_TcPlcDeviceInput.wDriveWcSt
ate

signal.
Connection monitoring, condition
monitoring.
Connection monitoring.

The function block deals with processing of the axis control value for output on an EL7031 stepper motor
output stage terminal.

192

Version: 1.8.3

TF5810

PLCopen Motion Control

I/O variable
STM Velocity.Velocity

Interface.Variable
ST_TcPlcDeviceOutput.nDacOut Operation: Output of the velocity

Use

signal.

STM Control.Control

ST_TcPlcDeviceOutput.uiDriveCtrl Operation: Control of the output

STM Status.Status

ST_TcPlcDeviceInput.uiStatus

WcState.WcState

InfoData.State

InfoData.AdsAddr

iTcMc_DriveEL7041

ST_TcPlcDeviceInput.wDriveWcSt
ate
ST_TcPlcDeviceInput.uiDriveBoxSt
ate
ST_TcPlcDeviceInput.sDrvAdsAdd
r

stage.
Operation: Status of the output
stage.
Connection monitoring.

Connection monitoring, condition
monitoring.
Communication.

The function block deals with processing of the axis control value for output on an EL7041 stepper motor
output stage terminal.

I/O variable
STM Velocity.Velocity

Interface.Variable
ST_TcPlcDeviceOutput.nDacOut Operation: Output of the velocity

Use

signal.

STM Control.Control

ST_TcPlcDeviceOutput.uiDriveCtrl Operation: Control of the output

STM Status.Status

ST_TcPlcDeviceInput.uiStatus

ENC Status.Counter Value

ST_TcPlcDeviceInput.uiCount

ENC Status.Latch Value

ST_TcPlcDeviceInput.uiLatch

ENC Status.Status

ENC Control.Control

WcState.WcState

InfoData.State

InfoData.AdsAddr

iTcMc_DriveEL7201

ST_TcPlcDeviceInput.uiTerminalSt
ate2
ST_TcPlcDeviceOutput.uiTerminal
Ctrl2
ST_TcPlcDeviceInput.wDriveWcSt
ate
ST_TcPlcDeviceInput.uiDriveBoxSt
ate
ST_TcPlcDeviceInput.sDrvAdsAdd
r

stage.
Operation: Status of the output
stage.
Operation: Read the actual
position.
Operation: Reading the latch
position.
Operation: Status of the encoder
interface.
Operation: Control of the encoder
interface.
Connection monitoring.

Connection monitoring, condition
monitoring.
Communication.

The function block prepares the control value of the axis for output to an EL7201 servo terminal.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions, particularly for parameter communication, overlap
with those of the encoder function block. See also iTcMc_EncoderEL7201.

TF5810

Version: 1.8.3

193

PLCopen Motion Control

I/O variable
Target velocity

Interface.Variable
ST_TcPlcDeviceOutput.NominalVe
lo

Use
Operation: Output of the velocity
signal.

Controlword

ST_TcPlcDeviceOutput.uiDriveCtrl Operation: Control of the output

Position actual value

ST_TcPlcDeviceInput.udiCount

WcState

Statusword

InfoData.State

InfoData.AdsAddr

iTcMc_DriveKL2521

ST_TcPlcDeviceInput.wDriveWcSt
ate
ST_TcPlcDeviceInput.uiStatus

ST_TcPlcDeviceInput.uiDriveBoxSt
ate
ST_TcPlcDeviceInput.sDrvAdsAdd
r

stage.
Operation: Read the actual
position.
Connection monitoring.

Operation: Status of the output
stage.
Connection monitoring, condition
monitoring.
Communication

The function block deals with processing of the axis control value for output on a KL2521 pulse output
terminal.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions, particularly for parameter communication, overlap
with those of the encoder function block. See also iTcMc_EncoderKL2521 [} 211].

I/O variable
Data in

Control

Status

Data out

Interface.Variable
ST_TcPlcDeviceInput.uiTerminalD
ata

Use
Operation: Read the actual
position.

For register communication
[} 336]: Interface for read data.
Register communication

ST_TcPlcDeviceOutput.bTerminal
Ctrl
ST_TcPlcDeviceInput.bTerminalSt
ate
ST_TcPlcDeviceOutput.nDacOut Operation: Output of the velocity

Register communication

signal.

Register communication: Interface
for written data.

iTcMc_DriveKL2531

The function block deals with processing of the axis control value for output on a KL2531 stepper motor
output stage terminal.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions, particularly for parameter communication, overlap
with those of the encoder function block. See also iTcMc_EncoderKL2531 [} 211].

194

Version: 1.8.3

TF5810

I/O variable
Velocity

Interface.Variable
ST_TcPlcDeviceOutput.nDacOut Operation: Output of the velocity

Use

PLCopen Motion Control

Position

Ctrl

Status

ExtStatus

ST_TcPlcDeviceInput.uiTerminalD
ata

ST_TcPlcDeviceOutput.bTerminal
Ctrl
ST_TcPlcDeviceInput.bTerminalSt
ate
ST_TcPlcDeviceInput.uiTerminalSt
ate2

signal.

For register communication
[} 336]: Interface for written data.
Operation: Read the actual
position.

For register communication:
Interface for read data.
Control the output stage, register
communication.
Status of the output stage, register
communication.
Diagnosis of output stage and
motor

iTcMc_DriveKL2532

The function block deals with processing of the axis control value for output on a KL2532 DC motor output
stage terminal.

I/O variable
Data in

Control

Status

Data out

Interface.Variable
ST_TcPlcDeviceInput.uiTerminalD
ata

ST_TcPlcDeviceOutput.bTerminal
Ctrl
ST_TcPlcDeviceInput.bTerminalSt
ate
ST_TcPlcDeviceOutput.nDacOut

Use

For register communication
[} 336]: Interface for read data.
Register communication.

Register communication

Register communication

iTcMc_DriveKL2535_1Coil, iTcMc_DriveKL2535_2Coil

The function block deals with processing of the axis control value for output on a KL2535 PWM output stage
terminal.

I/O variable
Data in

Control

Status

Data out

iTcMc_DriveKL2541

Interface.Variable
ST_TcPlcDeviceInput.uiTerminalD
ata
ST_TcPlcDeviceOutput.bTerminal
Ctrl
ST_TcPlcDeviceInput.bTerminalSt
ate
ST_TcPlcDeviceOutput.nDacOut

Use

Register communication [} 336]

Register communication.

Register communication

Register communication

The function block deals with processing of the axis control value for output on a KL2541 stepper motor
output stage terminal.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions, particularly for parameter communication, overlap
with those of the encoder function block. See also iTcMc_EncoderKL2541 [} 212].

TF5810

Version: 1.8.3

195

PLCopen Motion Control

I/O variable
Velocity

Interface.Variable
ST_TcPlcDeviceOutput.nDacOut Operation: Output of the velocity

Use

Position

Ctrl

Status

ExtCtrl

ExtStatus

ST_TcPlcDeviceInput.uiTerminalD
ata

ST_TcPlcDeviceOutput.bTerminal
Ctrl
ST_TcPlcDeviceInput.bTerminalSt
ate
ST_TcPlcDeviceOutput.uiTerminal
Ctrl2

ST_TcPlcDeviceInput.uiTerminalSt
ate2

signal.

For register communication
[} 336]: Interface for written data.
Operation: Read the actual
position.

For register communication:
Interface for read data.
Control the output stage, register
communication.
Status of the output stage, register
communication.
Latch control during homing with
the synchronous pulse of the
encoder
Diagnosis of output stage and
motor, latch status during homing
with the synchronous pulse of the
encoder

iTcMc_DriveKL2542

The function block deals with processing of the axis control value for output on a KL2542 DC motor output
stage terminal.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions, particularly for parameter communication, overlap
with those of the encoder function block. See also iTcMc_EncoderKL2542 [} 212].

I/O variable
Data out

Interface.Variable
ST_TcPlcDeviceOutput.nDacOut Operation: Output of the velocity

Use

Data in

Control

Status

ST_TcPlcDeviceInput.uiTerminalD
ata

ST_TcPlcDeviceOutput.bTerminal
Ctrl
ST_TcPlcDeviceInput.bTerminalSt
ate

signal.

For register communication
[} 336]: Interface for written data.
Operation: Read the actual
position.

For register communication:
Interface for read data.
Control the output stage, register
communication.
Status of the output stage, register
communication.

iTcMc_DriveKL4032

The function block deals with processing of the axis control value for output on a ±10 V output terminal.

196

Version: 1.8.3

TF5810

I/O variable
Data out

Interface.Variable
ST_TcPlcDeviceOutput.nDacOut Operation: Output of the velocity

Use

PLCopen Motion Control

Control

Status

Data in

ST_TcPlcDeviceOutput.bTerminal
Ctrl
ST_TcPlcDeviceInput.bTerminalSt
ate
ST_TcPlcDeviceOutput.uiTerminal
Data

signal.

For register communication
[} 336]: Interface for written data.
Register communication

Register communication

Register communication: Interface
for read data.

iTcMc_DriveLowCostStepper

The function block deals with processing of the axis control value for output on digital output terminals. For
emulation of an actual position, a pulse counter is updated, which can be evaluated with an
iTcMc_EncoderLowCostStepper [} 214] encoder.

I/O variable
Output

Output
Output

Output

Interface.Variable
ST_TcPlcDeviceOutput.nDigOutAp Non-inverted control of the A

Use

phase.

ST_TcPlcDeviceOutput.nDigOutAn Inverted control of the A phase.
ST_TcPlcDeviceOutput.nDigOutBp Non-inverted control of the B

phase.

ST_TcPlcDeviceOutput.nDigOutBn Inverted control of the B phase.

iTcMc_DriveLowCostInverter

The function block deals with processing of the axis control value for output on digital output terminals for
operation of a pole reversing contactor configuration or a frequency inverter with fixed frequencies. If this
drive type is used, a number of special characteristics must be taken into account. For linking, a distinction
has to be made between two options:

Brake, enable, direction and velocity level

After the MC_AxRtFinish_BkPlcMc [} 246] or MC_AxStandardBody_BkPlcMc [} 253]function block of the axis
has been called, four decoded signals are available. In order to generate the required signals, the following
consolidations of the direction-specific signals are required after the function block call.
Sample:
stAxDeviceOut.bDigOutAp:=stAxDeviceOut.bDigOutAp OR stAxDeviceOut.bDigOutBp;

stAxDeviceOut.bDigOutAn:=stAxDeviceOut.bDigOutAn OR stAxDeviceOut.bDigOutBn;

From V3.0.11 the output of an absolute value can be activated on the valve tab. In this case, the
signal consolidation shown above is applied internally.

TF5810

Version: 1.8.3

197

PLCopen Motion Control

I/O variable
Output

Interface.Variable
ST_TcPlcDeviceOutput.nDigOutAp Selection of the fixed frequency for

Use

Output

Output

Output

Output
Output
Input

rapid traverse.

ST_TcPlcDeviceOutput.nDigOutAn Selection of the fixed frequency for

slow traverse.

ST_TcPlcDeviceOutput.bMovePos Specifies the direction of travel:

Positive.

ST_TcPlcDeviceOutput.bMoveNeg Specifies the direction of travel:

Negative.

ST_TcPlcDeviceOutput.bPowerOn Enabling the power stage.
ST_TcPlcDeviceOutput.bBrakeOff Activation of the brake.
ST_TcPlcDeviceInput.bPowerOk

Status of the converter: Ready for
operation.

Brake, enable and direction-coded velocity level

I/O variable
Output

Interface.Variable
ST_TcPlcDeviceOutput.nDigOutAp Selection of the fixed frequency for

Use

Output

Output

Output

Output
Output
Input

rapid traverse in positive direction
of travel.

ST_TcPlcDeviceOutput.nDigOutAn Selection of the fixed frequency for

slow traverse in positive direction
of travel.

ST_TcPlcDeviceOutput.nDigOutBn Selection of the fixed frequency for
slow traverse in negative direction
of travel.

ST_TcPlcDeviceOutput.nDigOutBp Selection of the fixed frequency for
rapid traverse in negative direction
of travel.

ST_TcPlcDeviceOutput.bPowerOn Enabling the power stage.
ST_TcPlcDeviceOutput.bBrakeOff Activation of the brake.
ST_TcPlcDeviceInput.bPowerOk

Status of the converter: Ready for
operation.

iTcMc_DriveM2400_Dn

The function block performs preparation of the control value for the axis so that it can be output on one of the
four channels of a ±10 V M2400 output box.

I/O variable
Data out

Interface.Variable
ST_TcPlcDeviceOutput.nDacOut Operation: Output of the velocity

Use

signal.

4.4.3

Encoder

4.4.3.1

MC_AxRtEncoder_BkPlcMc

Available from version 3.0

198

Version: 1.8.3

TF5810

MC_AxRtEncoder_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ErrorUDINT  ErrorIdBOOL  AboveLimitBOOL  BelowLimitThis function block determines the actual position of the axis from the input information of a hardware
module. To this end a function block is called depending on the value set as nEnc_Type in
Axis.ST_TcHydAxParam [} 130], which takes into account the special features of the hardware module.

MC_AxRtHybridAxisActuals_BkPlcMc [} 224] is an adapted function block for determining the essential actual
values of a servo-electric/hydraulic hybrid axis.

PLCopen Motion Control

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Error:          BOOL;
    ErrorID:        UDINT;
    AboveLimit:     BOOL;
    BelowLimit:     BOOL;
END_VAR

Name
Error
ErrorID
AboveLimit
BelowLimit

Type
BOOL
UDINT
BOOL
BOOL

Description
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.
Exceeding of the upper software limit switch is indicated by the actual position.
If the value falls below the lower software limit switch, this is indicated by the
actual position.

Behavior of the function block

The function block investigates the axis interface that has been passed to it every time it is called. A number
of problems can be detected and reported during this process:

• If nEnc_Type in pStAxParams is set to an unacceptable value, the function block responds with Error

and ErrorID:=dwTcHydErrCdEncType. The axis is set to an error state.

• If one of the specific sub-function-blocks detects a problem, it will (if possible) place the axis into a fault

state. This error is then echoed at the outputs of the MC_AxRtEncoder_BkPlcMc.

If it is possible to carry out these checks without encountering any problems, the actual value of the axis is
determined by calling a type-specific function block corresponding to the nEnc_Type [} 98] in
Axis.ST_TcHydAxParam [} 130].

Information about the necessary linking of I/O components with the input and output structures of the axis
may be found in the Knowledge Base under FAQ #4 [} 324].

If only the usual blocks (encoder, generator, finish, drive) for the axis are to be called, a block of type
MC_AxStandardBody_BkPlcMc [} 253] should be used for simplicity.

The function blocks MC_AxUtiReadRegEncTerm_BkPlcMc [} 294] and MC_AxUtiWriteRegEncTerm_BkPlcMc
[} 303] are available for asynchronous data exchange with I/O devices of the KL series.

iTcMc_EncoderAx2000_B110A

The function block handles the evaluation of the actual values of an AX2000 servo actuator at the EtherCAT
fieldbus. This assumes that the connected motor is equipped with an absolute encoder. If a motor is
operated with a resolver, iTcMc_EncoderAx2000_B110R should be set.

TF5810

Version: 1.8.3

199

PLCopen Motion Control

During manual insertion or automatic detection of a drive actuator the TwinCAT System Manager
will suggest to insert an NC axis in the project and connect it with this actuator. If this actuator is to
be controlled with the hydraulic system library, it is essential to decline this proposition.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions overlap with those of the drive function block. See
also iTcMc_DriveAX2000_B110R [} 188].

I/O variable
Position actual value

Status word
Control word
Velocity demand value

WcState (see note)

WcState

InfoData.State

InfoData.AdsAddr (see note)

InfoData.AdsAddr

Chn0 (see note)

Chn0

Output (on a DO terminal)

Output of the velocity control value.

Device status, encoder emulation.

Use
Determines the actual position.

Interface.Variable
ST_TcPlcDeviceInput.ActualPos[0.
.1]
ST_TcPlcDeviceInput.uiStatus
ST_TcPlcDeviceOutput.uiDriveCtrl Device control.
ST_TcPlcDeviceOutput.NominalVe
lo
ST_TcPlcDeviceInput.wEncWcStat
e
ST_TcPlcDeviceInput.wDriveWcSt
ate
ST_TcPlcDeviceInput.uiDriveBoxSt
ate
ST_TcPlcDeviceInput.sEncAdsAdd
r
ST_TcPlcDeviceInput.sDrvAdsAdd
r
ST_TcPlcDeviceInput.nEncAdsCha
nnel
ST_TcPlcDeviceInput.nDrvAdsCha
nnel
ST_TcPlcDeviceOutput.PowerOn Optional control of the mains

Control of real-time status,
parameter communication.
Parameter communication.

Connection monitoring for actual
value acquisition.
Connection monitoring for the
drive.
Monitoring of online status

Control of real-time status,
parameter communication.

Parameter communication.

Input (on a DI terminal)

ST_TcPlcDeviceInput.PowerOk

contactor. A digital output terminal
is required for this purpose.
Optional evaluation of the mains
contactor. A digital input terminal is
required for this purpose.

In order to simplify the establishment of the I/O link, the linking of
ST_TcPlcDeviceInput.sEncAdsAddr, ST_TcPlcDeviceInput.nEncAdsChannel and
ST_TcPlcDeviceInput.wEncWcState can be avoided, if the actual value acquisition takes place via
the same device, as usual. In this case, the function blocks for parameter communication and
encoder evaluation use the corresponding variables of the drive link.

iTcMc_EncoderAx2000_B110R

The function block handles the evaluation of the actual values of an AX2000 servo actuator at the EtherCAT
fieldbus. This assumes that the connected motor is equipped with a resolver. If a motor is operated with an
absolute encoder, iTcMc_EncoderAx2000_B110A must be set.

During manual insertion or automatic detection of a drive actuator the TwinCAT System Manager
will suggest to insert an NC axis in the project and connect it with this actuator. If this actuator is to
be controlled with the hydraulic system library, it is essential to decline this proposition.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions overlap with those of the drive function block. See
also iTcMc_DriveAX2000_B110R [} 188].

200

Version: 1.8.3

TF5810

PLCopen Motion Control

I/O variable
Position actual value

Status word
Control word
Velocity demand value

WcState (see note)

WcState

uiDriveBoxState

InfoData.AdsAddr (see note)

InfoData.AdsAddr

Chn0 (see note)

Chn0

Output (on a DO terminal)

Output of the velocity control value.

Device status, encoder emulation.

Use
Determines the actual position.

Interface.Variable
ST_TcPlcDeviceInput.ActualPos[0.
.1]
ST_TcPlcDeviceInput.uiStatus
ST_TcPlcDeviceOutput.uiDriveCtrl Device control.
ST_TcPlcDeviceOutput.NominalVe
lo
ST_TcPlcDeviceInput.wEncWcStat
e
ST_TcPlcDeviceInput.wDriveWcSt
ate
ST_TcPlcDeviceInput.InfoData.Stat
e
ST_TcPlcDeviceInput.sEncAdsAdd
r
ST_TcPlcDeviceInput.sDrvAdsAdd
r
ST_TcPlcDeviceInput.nEncAdsCha
nnel
ST_TcPlcDeviceInput.nDrvAdsCha
nnel
ST_TcPlcDeviceOutput.PowerOn Optional control of the mains

Connection monitoring for actual
value acquisition.
Connection monitoring for the
drive.
Monitoring of online status

Parameter communication.

Parameter communication.

Parameter communication.

Parameter communication.

Input (on a DI terminal)

ST_TcPlcDeviceInput.PowerOk

contactor. A digital output terminal
is required for this purpose.
Optional evaluation of the mains
contactor. A digital input terminal is
required for this purpose.

In order to simplify the establishment of the I/O link, the linking of
ST_TcPlcDeviceInput.sEncAdsAddr, ST_TcPlcDeviceInput.nEncAdsChannel and
ST_TcPlcDeviceInput.wEncWcState can be avoided, if the actual value acquisition takes place via
the same device, as usual. In this case, the function blocks for parameter communication and
encoder evaluation use the corresponding variables of the drive link.

iTcMc_EncoderAx2000_B200R, iTcMc_EncoderAx2000_B900R

The function block deals with evaluation of the actual values of an AX2000 servo actuator with Lightbus
(B200) or RealtimeEthernet (B900).

During manual insertion or automatic detection of a drive actuator the TwinCAT System Manager
will suggest to insert an NC axis in the project and connect it with this actuator. If this actuator is to
be controlled with the hydraulic system library, it is essential to decline this proposition.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions overlap with those of the drive function block. See
also iTcMc_DriveAX2000_B200R [} 189].

TF5810

Version: 1.8.3

201

PLCopen Motion Control

I/O variable
ActualPos[0..1]

DriveError
DriveState[0..3]

BoxState

DriveCtrl0
DriveCtrl1
DriveCtrl2
DriveCtrl3
NominalVelo

Output (on a DO terminal)

Use
Determines the actual position.

Device status.
Device status.

Interface.Variable
ST_TcPlcDeviceInput.ActualPos[0.
.1]
ST_TcPlcDeviceInput.DriveError
ST_TcPlcDeviceInput.DriveState[0.
.3]
ST_TcPlcDeviceInput.uiDriveBoxSt
ate
ST_TcPlcDeviceOutput.DriveCtrl[0] Device control.
ST_TcPlcDeviceOutput.DriveCtrl[1] Device control.
ST_TcPlcDeviceOutput.DriveCtrl[2] Device control.
ST_TcPlcDeviceOutput.DriveCtrl[3] Device control.
ST_TcPlcDeviceOutput.NominalVe
lo
ST_TcPlcDeviceOutput.PowerOn Optional control of the mains

Connection monitoring.

Output of the velocity control value.

Input (on a DI terminal)

ST_TcPlcDeviceInput.PowerOk

contactor. A digital output terminal
is required for this purpose.
Optional evaluation of the mains
contactor. A digital input terminal is
required for this purpose.

iTcMc_EncoderAx2000_B750A

The function block handles (from V3.0.26) the evaluation of the actual values of an AX2000 servo drive at
the Sercos fieldbus. This assumes that the connected motor is equipped with an absolute encoder.

During manual insertion or automatic detection of a drive actuator the TwinCAT System Manager
will suggest to insert an NC axis in the project and connect it with this actuator. If this actuator is to
be controlled with the hydraulics library, it is essential to decline this proposition.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions overlap with those of the drive function block. See
also iTcMc_DriveAX2000_B750A [} 189].

I/O variable
Drive status word
Actual position value encoder 1
Master control word
Velocity command value

SystemStatus (from Sercos
master)

Use
Device status.
Determines the actual position.

Interface.Variable
ST_TcPlcDeviceInput.uiStatus
ST_TcPlcDeviceInput.udiCount
ST_TcPlcDeviceOutput.uiDriveCtrl Device control.
ST_TcPlcDeviceOutput.NominalVe
lo
ST_TcPlcDeviceInput.uiDriveBoxSt
ate

Monitoring of the Sercos phase.
This variable is provided by the
Sercos master (e.g. FC7501).

Output of the velocity control value.

Output (on a DO terminal)

ST_TcPlcDeviceOutput.PowerOn Optional control of the mains

Input (on a DI terminal)

ST_TcPlcDeviceInput.PowerOk

contactor. A digital output terminal
is required for this purpose.
Optional evaluation of the mains
contactor. A digital input terminal is
required for this purpose.

Note a number of special characteristics. Further information can be found in the Knowledge Base [} 320].

iTcMc_EncoderAx5000_B110A, iTcMc_EncoderAx5000_B110SR

The function block handles the evaluation of the actual values of an AX5000 servo actuator at the EtherCAT
fieldbus. This assumes that the connected motor is equipped with an absolute encoder. If a motor is
operated with a resolver, iTcMc_EncoderAx5000_B110SR should be set.

202

Version: 1.8.3

TF5810

PLCopen Motion Control

During manual insertion or automatic detection of a drive actuator the TwinCAT System Manager
will suggest to insert an NC axis in the project and connect it with this actuator. If this actuator is to
be controlled with the hydraulic system library, it is essential to decline this proposition.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions overlap with those of the drive function block. See
also iTcMc_DriveAX5000_B110A [} 189].

I/O variable
Position feedback 1 value
Drive status word
Master control word
Velocity command value

WcState

WcState (see note)

InfoData.State

InfoData.AdsAddr

InfoData.AdsAddr (see note)

Chn0 (see note 2)

Output of the velocity control value.

Use
Interface.Variable
Determines the actual position.
ST_TcPlcDeviceInput.udiCount
ST_TcPlcDeviceInput.uiStatus
Device status.
ST_TcPlcDeviceOutput.uiDriveCtrl Device control.
ST_TcPlcDeviceOutput.NominalVe
lo
ST_TcPlcDeviceInput.wDriveWcSt
ate
ST_TcPlcDeviceInput.wEncWcStat
e
ST_TcPlcDeviceInput.
uiDriveBoxState
ST_TcPlcDeviceInput.sDrvAdsAdd
r
ST_TcPlcDeviceInput.sEncAdsAdd
r
ST_TcPlcDeviceInput.nDrvAdsCha
nnel

Connection monitoring for the
drive.
Connection monitoring for actual
value acquisition.
Monitoring of online status

Control of real-time status,
parameter communication.
Parameter communication.

For single devices or the first drive
of a dual device: Control of real-
time status, parameter
communication.
For single devices or the first drive
of a dual device: Parameter
communication.
Only for the second drive of a dual
device: Control of real-time status,
parameter communication.
Only for the second drive of a dual
device: Parameter communication.

Chn0 (see notes 1,2)

ST_TcPlcDeviceInput.nEncAdsCha
nnel

Chn1 (see note 2)

ST_TcPlcDeviceInput.nDrvAdsCha
nnel

Chn1 (see notes 1,2)

Output (on a DO terminal)

ST_TcPlcDeviceInput.nEncAdsCha
nnel
ST_TcPlcDeviceOutput.PowerOn Optional control of the mains

Input (on a DI terminal)

ST_TcPlcDeviceInput.PowerOk

contactor. A digital output terminal
is required for this purpose.
Optional evaluation of the mains
contactor. A digital input terminal is
required for this purpose.

The following list of compatible devices is naturally incomplete. It is not a recommendation but is merely
intended for information. Beckhoff cannot guarantee trouble-free operation of the listed devices. If a
manufacturer or one of their devices is not listed, trouble-free operation may well be possible, but is not
guaranteed.

Manufacturer
Baumüller

Type
b-maxx

Description
Servo controller with single-turn
absolute encoder

TF5810

Version: 1.8.3

203

PLCopen Motion Control

In order to simplify the establishment of the I/O link, the linking of
ST_TcPlcDeviceInput.sEncAdsAddr, ST_TcPlcDeviceInput.nEncAdsChannel and
ST_TcPlcDeviceInput.wEncWcState can be avoided, if the actual value acquisition takes place via
the same device, as usual. In this case, the function blocks for parameter communication and
encoder evaluation use the corresponding variables of the drive link.

The variables Chn0 and Chn2 are used for distinguishing the channels of a dual unit. Connect Chn0
for the first drive of the device and Chn1 for the second. For single devices proceed as for the first
channel of a dual device.

Note a number of special characteristics. Further information can be found in the Knowledge Base.

iTcMc_EncoderCoE_DS402A

The function block handles the evaluation of the actual values of a servo actuator with CoE DS402 profile at
the EtherCAT fieldbus. This assumes that the connected motor is equipped with a multi-turn absolute
encoder. AX8000 devices with absolute encoder support this profile.

During manual insertion or automatic detection of a drive actuator the TwinCAT System Manager
will suggest to insert an NC axis in the project and connect it with this actuator. If this actuator is to
be controlled with the hydraulics library, it is essential to decline this proposition.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions overlap with those of the drive function block. See
also iTcMc_DriveCoE_DS402 [} 189].

I/O variable
see notice

WcState

InfoData.State

InfoData.AdsAddr

Interface.Variable
ST_TcPlcDeviceInput.udiCount
ST_TcPlcDeviceInput.uiStatus
ST_TcPlcDeviceOutput.uiDriveCtrl
ST_TcPlcDeviceOutput.NominalVe
lo
ST_TcPlcDeviceInput.wDriveWcSt
ate
ST_TcPlcDeviceInput.uiDriveBoxSt
ate
ST_TcPlcDeviceInput.sEncAdsAdd
r

Use
Determines the actual position.

Connection monitoring.

Monitoring of online status

Automatic identification.

The names of the process data exchanged with the device are specified via the XML file of the
manufacturer.

A list with compatible devices can be found below.

Mapping Note AX8000:

204

Version: 1.8.3

TF5810

I/O variable
Position actual value
Statusword
Controlword
Target velocity

WcState

InfoData.State

InfoData.AdsAddr

Interface.Variable
ST_TcPlcDeviceInput.udiCount
ST_TcPlcDeviceInput.uiStatus
ST_TcPlcDeviceOutput.uiDriveCtrl
ST_TcPlcDeviceOutput.NominalVe
lo
ST_TcPlcDeviceInput.wDriveWcSt
ate
ST_TcPlcDeviceInput.uiDriveBoxSt
ate
ST_TcPlcDeviceInput.sEncAdsAdd
r

PLCopen Motion Control

Use
Determines the actual position.

Set velocity

Connection monitoring.

Monitoring of online status

Automatic identification.

iTcMc_EncoderCoE_DS402SR

The function block handles the evaluation of the actual values of a servo actuator with CoE DS402 profile at
the EtherCAT fieldbus. This assumes that the connected motor is equipped with a resolver or a single-turn
absolute encoder.

During manual insertion or automatic detection of a drive actuator the TwinCAT System Manager
will suggest to insert an NC axis in the project and connect it with this actuator. If this actuator is to
be controlled with the hydraulic system library, it is essential to decline this proposition.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions overlap with those of the drive function block. See
also iTcMc_DriveCoE_DS402 [} 189].

I/O variable
see note

WcState

InfoData.State

InfoData.AdsAddr

Interface.Variable
ST_TcPlcDeviceInput.udiCount
ST_TcPlcDeviceInput.uiStatus
ST_TcPlcDeviceOutput.uiDriveCtrl
ST_TcPlcDeviceOutput.NominalVe
lo
ST_TcPlcDeviceInput.wDriveWcSt
ate
ST_TcPlcDeviceInput.uiDriveBoxSt
ate
ST_TcPlcDeviceInput.sEncAdsAdd
r

Use
Determines the actual position.

Connection monitoring.

Monitoring of online status

Automatic identification.

The names of the process data exchanged with the device are specified via the XML file of the
manufacturer.

The encoder must support the following Index.SubIndex combinations.

TF5810

Version: 1.8.3

205

PLCopen Motion Control

Index
1000
1008
1018
1018
6080

608F

6090

Subindex
0
0
1
2
0

1

1

Meaning
Identification
Device name (optional)
Manufacturer ID
Device type
Maximum speed in RPM (optional;
if this object is not supported, the
reference speed must be entered
manually).
Number of encoder increments per
motor revolution.
Number of increments per motor
revolution used for control value
output.

The following list of compatible devices is naturally incomplete. It is not a recommendation but is merely
intended for information. Beckhoff cannot guarantee trouble-free operation of the listed devices. If a
manufacturer or one of their devices is not listed, trouble-free operation may well be possible, but is not
guaranteed.

Manufacturer
LTi DRiVES GmbH

Type

iTcMc_EncoderCoE_DS406

Description
Servo controller with single-turn
absolute encoder

The function block handles the evaluation of encoders with direct EtherCAT connection. The encoder must
support the CiA DS406 profile.

I/O variable
see note
see notes

WcState

InfoData.State

InfoData.AdsAddr

Interface.Variable
ST_TcPlcDeviceInput.udiCount
ST_TcPlcDeviceInput.wEncDevSta
te
ST_TcPlcDeviceInput.wEncWcStat
e
ST_TcPlcDeviceInput.uiEncBoxSta
te
ST_TcPlcDeviceInput.sEncAdsAdd
r

Use
Determines the actual position.
Monitoring the device status.

Connection monitoring.

Monitoring of online status.

Automatic identification.

The names of the process data exchanged with the device are specified via the XML file of the
manufacturer.

Monitoring of the device status is not guaranteed for all devices from all manufacturers. For some
devices an 8-bit status is provided. This kind of information should be mapped on the lower 8 bits of
the wEncDevState element.

The encoder must support the following Index.SubIndex combinations.

206

Version: 1.8.3

TF5810

Index
1000
1008
1018
1018
6001

6002

6005

6501

6502

650A

650B

Subindex
0
0
1
2
0

0

1

0

0

2

3

PLCopen Motion Control

Meaning
Identification
Device name (optional)
Manufacturer ID
Device type
Rotational encoders: increments
per revolution (obligatory)
Rotational encoders: Total counting
range (option A, alternatively: index
6502)

Linear encoders: Total counting
range (obligatory)
Linear encoders: Resolution
(option A, alternatively: index 6501)
Linear encoders: Resolution
(option B, alternatively: index 6005)
Rotational encoders: Number of
counted revolutions (option B,
alternatively: index 6002)
Linear encoders: lower limit of the
intended working area (option)
Linear encoders: upper limit of the
intended working area (option)

The following list of compatible devices is naturally incomplete. It is not a recommendation but is merely
intended for information. Beckhoff cannot guarantee trouble-free operation of the listed devices. If a
manufacturer or one of their devices is not listed, trouble-free operation may well be possible, but is not
guaranteed.

Certain parameters can be determined automatically, depending on the support of the listed objects. This
applies to the counting range, the overflow detection and (for linear encoders) the resolution. If the
respective objects are not provided or not in a supported combination, this is not possible. In such a case,
operation may be possible. However, the parameters must then be set manually during commissioning.

Manufacturer
Fritz Kübler GmbH
IVO GmbH & Co. KG
MTS
TR Electronic GmbH:
TWK-Electronic GmbH

iTcMc_EncoderDigCam

Type
58x8
GXMMW_H
Temposonics R
LMP
CRKxx12R12C1xx

Description
Multi-turn absolute encoder.
Multi-turn absolute encoder.
Linear absolute encoder.
Linear absolute encoder.
Multi-turn absolute encoder.

The function block handles the evaluation of four digital inputs as position cams.

I/O variable
Input

Input

Input

Input

Interface.Variable
ST_TcPlcDeviceInput.bDigCamPP Determines the actual position:

Use

Positive target cam.

ST_TcPlcDeviceInput.bDigCamP Determines the actual position:

Positive brake cam.

ST_TcPlcDeviceInput.bDigCamM Determines the actual position:

Negative brake cam.

ST_TcPlcDeviceInput.bDigCamMM Determines the actual position:

Negative target cam.

TF5810

Version: 1.8.3

207

PLCopen Motion Control

iTcMc_EncoderDigIncrement

The function block handles the evaluation of two digital inputs for the emulation of an incremental encoder
evaluation.

I/O variable
Input
Input

iTcMc_EncoderEL3102

Interface.Variable
ST_TcPlcDeviceInput.bDigInA
ST_TcPlcDeviceInput.bDigInB

Use
Determines the actual position.
Determines the actual position.

The function block handles the evaluation of data from an EL3102 analog input terminal.

I/O variable
Value
InfoData.AdsAddr

InfoData.State

WcState

iTcMc_EncoderEL3142

Interface.Variable
ST_TcPlcDeviceInput.uiCount
ST_TcPlcDeviceInput.sEncAdsAdd
r
ST_TcPlcDeviceInput.uiEncBoxSta
te
ST_TcPlcDeviceInput.wEncWcStat
e

Use
Read the actual position.
Optional: Address information for
parameter communication via CoE.
Connection monitoring, condition
monitoring.
Connection monitoring.

The function block handles the evaluation of data from an EL3142 analog input terminal. The mapping is
similar to the interface-compatible EL3102.

iTcMc_EncoderEL3162

The function block handles the evaluation of data from an EL3162 analog input terminal. The mapping is
similar to the interface-compatible EL3102.

iTcMc_EncoderEL3255

The function block handles the evaluation of data from an EL3255 analog input terminal.

I/O variable
AI Standard Channel x.Value
AI Standard Channel x.Status

InfoData.AdsAddr

InfoData.State

WcState

Interface.Variable
ST_TcPlcDeviceInput.uiCount
ST_TcPlcDeviceInput.wEncDevSta
te
ST_TcPlcDeviceInput.sEncAdsAdd
r
ST_TcPlcDeviceInput.uiEncBoxSta
te
ST_TcPlcDeviceInput.wEncWcStat
e

Use
Read the actual position.
Evaluation of the fault signal of the
encoder.
Address information for parameter
communication via CoE.
Connection monitoring, condition
monitoring.
Connection monitoring.

The terminal supports up to five encoders. The variables InfoData.AdsAddr, InfoData.State and
WcState should be distributed to all axes involved through multiple mapping.

iTcMc_EncoderEL5001

The function block handles the evaluation of data from an EL5001 SSI encoder terminal.

208

Version: 1.8.3

TF5810

PLCopen Motion Control

Interface.Variable
ST_TcPlcDeviceInput.udiCount
ST_TcPlcDeviceOutput.usiRegStat
us
ST_TcPlcDeviceInput.sEncAdsAdd
r
ST_TcPlcDeviceInput.uiEncBoxSta
te
ST_TcPlcDeviceInput.wEncWcStat
e

Use
Read the actual position.
Evaluation of the fault signal of the
encoder.
Address information for parameter
communication via CoE.
Connection monitoring, condition
monitoring.
Connection monitoring.

I/O variable
Value
Status

InfoData.AdsAddr

InfoData.State

WcState

iTcMc_EncoderEL5021

The function block handles the evaluation of data from an EL5021 sin/cos encoder terminal.

I/O variable
ENC Status.Counter value
ENC Status.Status

ENC Status.Latch value

ENC Control.Control
InfoData.AdsAddr

InfoData.State

WcState

Interface.Variable
ST_TcPlcDeviceInput.udiCount
ST_TcPlcDeviceInput.usiRegStatu
s
ST_TcPlcDeviceInput.udiLatch

ST_TcPlcDeviceOutput.usiCtrl
ST_TcPlcDeviceInput.sEncAdsAdd
r
ST_TcPlcDeviceInput.uiEncBoxSta
te
ST_TcPlcDeviceInput.wEncWcStat
e

Use
Read the actual position.
Evaluation of the fault signal of the
encoder.
For homing using the synchronous
pulse of the encoder.
Control of the latch function.
Address information for parameter
communication via CoE.
Connection monitoring, condition
monitoring.
Connection monitoring.

iTcMc_EncoderEL5032 (ab V3.0.40)

The function block handles the evaluation of data from an EL5032 ENDAT encoder terminal.

The EL5032 terminal provides a 32-bit or 64-bit counter, depending on its setting. This means that the
highest value that can be displayed is either 232 – 1 or 264 – 1. Multiplied with the encoder resolution, this
results in the evaluable path. At 10 nm resolution results in a value of 42949 mm. This is sufficient for most
applications, which is why it is usually OK to use the terminal in 32-bit mode. To do this, only the mapping to
udiCount is required. Otherwise, the 64-bit mode of the terminal must be activated and the complete
mapping to udiCount and S_DiReserve[1] must be configured.

Note the supply voltage

NOTICE

To prevent damage to the connected device, check the supply voltage set in the EL5032 before connecting
the device

When a fieldbus is started and an axis error is reset, certain parameters of the connected device are read.
The device type is included in the logging. Only absolute linear scales and absolute multi-turn encoders are
accepted. With linear scales, the resolution is automatically updated in the encoder weighting and
interpolation.

TF5810

Version: 1.8.3

209

PLCopen Motion Control

I/O variable
Position (DWORD or lower part of
ULINT)
Position (upper part of ULINT)

Position (upper part of ULINT)

Status

InfoData.AdsAddr

InfoData.State

WcState

iTcMc_EncoderEL5101

Interface.Variable
ST_TcPlcDeviceInput.udiCount

Use
Read the actual position.

ST_TcPlcDeviceInput.S_DiReserv
e[1]
ST_TcPlcDeviceInput.udiLatch

ST_TcPlcDeviceInput.uiEncDevSta
te
ST_TcPlcDeviceInput.sEncAdsAdd
r
ST_TcPlcDeviceInput.uiEncBoxSta
te
ST_TcPlcDeviceInput.wEncWcStat
e

Optional: Reading of the actual
position under TwinCAT 2.
Optional: Reading of the actual
position under TwinCAT 3.
Evaluation of the fault signal of the
encoder.
Address information for parameter
communication via CoE.
Connection monitoring, condition
monitoring.
Connection monitoring.

The function block handles the evaluation of data from an EL5101 incremental encoder terminal.

I/O variable
Value

Latch

Ctrl
Status

InfoData.AdsAddr

InfoData.State

WcState

Interface.Variable
ST_TcPlcDeviceInput.uiCount

ST_TcPlcDeviceInput.uiLatch

ST_TcPlcDeviceOutput.usiCtrl
ST_TcPlcDeviceInput.usiStatus

ST_TcPlcDeviceInput.sEncAdsAdd
r
ST_TcPlcDeviceInput.uiEncBoxSta
te
ST_TcPlcDeviceInput.wEncWcStat
e

Use
Operation: Read the actual
position.
For homing using the synchronous
pulse of the encoder.
Control of the latch function etc.
Status of the encoder, of the latch
function.
Address information for parameter
communication via CoE.
Connection monitoring, condition
monitoring.
Connection monitoring.

iTcMc_EncoderEL5111

The function block handles the evaluation of data from an EL5111 incremental encoder terminal.

I/O variable
Value

Latch

Ctrl
Status

InfoData.AdsAddr

InfoData.State

WcState

Interface.Variable
ST_TcPlcDeviceInput.uiCount

ST_TcPlcDeviceInput.uiLatch

ST_TcPlcDeviceOutput.usiCtrl
ST_TcPlcDeviceInput.usiStatus

ST_TcPlcDeviceInput.sEncAdsAdd
r
ST_TcPlcDeviceInput.uiEncBoxSta
te
ST_TcPlcDeviceInput.wEncWcStat
e

Use
Operation: Read the actual
position.
For homing using the synchronous
pulse of the encoder.
Control of the latch function etc.
Status of the encoder, of the latch
function.
Address information for parameter
communication via CoE.
Connection monitoring, condition
monitoring.
Connection monitoring.

iTcMc_EncoderEL7041

The function block handles the evaluation of data from an EL7041 stepper motor output terminal.

210

Version: 1.8.3

TF5810

PLCopen Motion Control

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions, particularly for parameter communication, overlap
with those of the drive function block. See also iTcMc_DriveEL7041 [} 193].

iTcMc_EncoderEL7201

The function block handles the evaluation of data from an EL7201 servo output terminal.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions, particularly for parameter communication, overlap
with those of the drive function block. See also iTcMc_DriveEL7201.

iTcMc_EncoderIx5009

The function block handles the evaluation of data from an IP5009 SSI encoder box.

I/O variable
PZDL_RegDaten

Interface.Variable
ST_TcPlcDeviceInput.uiPZDL_Reg
Daten

Use
Operation: Read the actual
position.

PZDH
RegStatus

ST_TcPlcDeviceInput.uiPZDH
ST_TcPlcDeviceInput.usiRegStatu
s

iTcMc_EncoderKL2521

For register communication
[} 336]: Interface for read data.
Read the actual position.
Miscellaneous status information.

The function block handles the evaluation of data from a KL2521 pulse output terminal. The output pulses
are counted and used for an encoder emulation.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions, particularly for parameter communication, overlap
with those of the drive function block. See also iTcMc_DriveKL2521 [} 194].

I/O variable
Data in

Control

Status

Data out

Interface.Variable
ST_TcPlcDeviceInput.uiTerminalD
ata

Use
Operation: Read the actual
position.

For register communication
[} 336]: Interface for read data.
Register communication

ST_TcPlcDeviceOutput.bTerminal
Ctrl
ST_TcPlcDeviceInput.bTerminalSt
ate
ST_TcPlcDeviceOutput.nDacOut Operation: Output of the velocity

Register communication

signal.

Register communication: Interface
for written data.

iTcMc_EncoderKL2531

The function block handles the evaluation of data from a KL2531 pulse output terminal. The output pulses
are counted and used for an encoder emulation.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions, particularly for parameter communication, overlap
with those of the drive function block. See also iTcMc_DriveKL2531 [} 194].

TF5810

Version: 1.8.3

211

PLCopen Motion Control

I/O variable
Velocity

Interface.Variable
ST_TcPlcDeviceOutput.nDacOut Operation: Output of the velocity

Use

Position

Ctrl

Status

ExtStatus

ST_TcPlcDeviceInput.uiTerminalD
ata

ST_TcPlcDeviceOutput.bTerminal
Ctrl
ST_TcPlcDeviceInput.bTerminalSt
ate
ST_TcPlcDeviceInput.uiTerminalSt
ate2

signal.

For register communication
[} 336]: Interface for written data.
Operation: Read the actual
position.

For register communication:
Interface for read data.
Control the output stage, register
communication.
Status of the output stage, register
communication.
Diagnosis of output stage and
motor

iTcMc_EncoderKL2541

The function block handles the evaluation of data from a KL2541 pulse output terminal. The output pulses
are counted and used for an encoder emulation.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions, particularly for parameter communication, overlap
with those of the drive function block. See also iTcMc_DriveKL2541 [} 195].

I/O variable
Velocity

Interface.Variable
ST_TcPlcDeviceOutput.nDacOut Operation: Output of the velocity

Use

Position

Ctrl

Status

ExtCtrl

ExtStatus

ST_TcPlcDeviceInput.uiTerminalD
ata

ST_TcPlcDeviceOutput.bTerminal
Ctrl
ST_TcPlcDeviceInput.bTerminalSt
ate
ST_TcPlcDeviceOutput.uiTerminal
Ctrl2

ST_TcPlcDeviceInput.uiTerminalSt
ate2

signal.

For register communication
[} 336]: Interface for written data.
Operation: Read the actual
position.

For register communication:
Interface for read data.
Control the output stage, register
communication.
Status of the output stage, register
communication.
Latch control during homing with
the synchronous pulse of the
encoder
Diagnosis of output stage and
motor, latch status during homing
with the synchronous pulse of the
encoder

iTcMc_EncoderKL2542

The function block handles the evaluation of data from a KL2542 motor output stage terminal.

This I/O device belongs to a group of devices, which are used for the control value output as well as actual
value determination. The required mapping definitions, particularly for parameter communication, overlap
with those of the drive function block. See also iTcMc_DriveKL2542 [} 196].

212

Version: 1.8.3

TF5810

I/O variable
Data out

Interface.Variable
ST_TcPlcDeviceOutput.nDacOut Operation: Output of the velocity

Use

PLCopen Motion Control

Data in

Control

Status

ST_TcPlcDeviceInput.uiTerminalD
ata

ST_TcPlcDeviceOutput.bTerminal
Ctrl
ST_TcPlcDeviceInput.bTerminalSt
ate

signal.

For register communication
[} 336]: Interface for written data.
Operation: Read the actual
position.

For register communication:
Interface for read data.
Control the output stage, register
communication.
Status of the output stage, register
communication.

iTcMc_EncoderKL3002

The function block handles the evaluation of data from a KL3002 analog input terminal.

I/O variable
Data in
Ctrl

Status

iTcMc_EncoderKL3042

Interface.Variable
ST_TcPlcDeviceInput.uiCount
ST_TcPlcDeviceOutput.usiCtrl

ST_TcPlcDeviceInput.usiStatus

Use
Read the actual position.

Register communication [} 336]
Register communication.

The function block handles the evaluation of data from a KL3042 analog input terminal.

I/O variable
Data in
Ctrl

Status

iTcMc_EncoderKL3062

Interface.Variable
ST_TcPlcDeviceInput.uiCount
ST_TcPlcDeviceOutput.usiCtrl

ST_TcPlcDeviceInput.usiStatus

Use
Read the actual position.

Register communication [} 336]
Register communication.

The function block handles the evaluation of data from a KL3062 analog input terminal.

I/O variable
Data in
Ctrl

Status

iTcMc_EncoderKL3162

Interface.Variable
ST_TcPlcDeviceInput.uiCount
ST_TcPlcDeviceOutput.usiCtrl

ST_TcPlcDeviceInput.usiStatus

Use
Read the actual position.

Register communication [} 336]
Register communication.

The function block handles the evaluation of data from a KL3162 analog input terminal.

I/O variable
Data in
Ctrl

Status

iTcMc_EncoderKL5001

Interface.Variable
ST_TcPlcDeviceInput.uiCount
ST_TcPlcDeviceOutput.usiCtrl

ST_TcPlcDeviceInput.usiStatus

Use
Read the actual position.

Register communication [} 336]
Register communication.

The function block handles the evaluation of data from a KL5001 SSI encoder terminal.

TF5810

Version: 1.8.3

213

PLCopen Motion Control

I/O variable
PZDL_RegDaten

PZDH
RegStatus

RegDaten

iTcMc_EncoderKL5101

Interface.Variable
ST_TcPlcDeviceInput.uiPZDL_Reg
Daten

Use
Operation: Read the actual
position.

ST_TcPlcDeviceInput.uiPZDH
ST_TcPlcDeviceInput.usiRegStatu
s
ST_TcPlcDeviceOutput.bTerminal
Data

For register communication
[} 336]: Interface for read data.
Read the actual position.
Miscellaneous status information.

Register communication.

The function block handles the evaluation of data from a KL5101 incremental encoder terminal.

I/O variable
Counter

Interface.Variable
ST_TcPlcDeviceInput.uiCount

Latch

Ctrl

Status
RegDaten

ST_TcPlcDeviceInput.uiLatch

ST_TcPlcDeviceOutput.usiCtrl

ST_TcPlcDeviceInput.usiStatus
ST_TcPlcDeviceOutput.bTerminal
Data

Use
Operation: Read the actual
position.

For register communication:
Interface for read data.
For homing using the synchronous
pulse of the encoder.
Control of the latch function etc.,
register communication [} 336]
Miscellaneous status information.
Register communication.

iTcMc_EncoderKL5111

The function block handles the evaluation of data from a KL5111 incremental encoder terminal.

I/O variable
Counter

Interface.Variable
ST_TcPlcDeviceInput.uiCount

Latch

Ctrl

Status
RegDaten

ST_TcPlcDeviceInput.uiLatch

ST_TcPlcDeviceOutput.usiCtrl

ST_TcPlcDeviceInput.usiStatus
ST_TcPlcDeviceOutput.bTerminal
Data

Use
Operation: Read the actual
position.

For register communication:
Interface for read data.
For homing using the synchronous
pulse of the encoder.
Control of the latch function etc.,
register communication [} 336]
Miscellaneous status information.
Register communication.

iTcMc_EncoderLowCostStepper

If the value iTcMc_DriveLowCostStepper [} 197] is entered as nDrive_Type, the half steps that are output are
counted in ST_TcPlcDeviceOutput.uiCount. The result is used to calculate the actual position. Mapping is
not required for the encoder.

This encoder type can only be used in combination with an iTcMc_DriveLowCostStepperdrive.

214

Version: 1.8.3

TF5810

PLCopen Motion Control

iTcMc_EncoderM2510

The function block handles the evaluation of data from an M2510 analog input box.

I/O variable
Data in

Interface.Variable
ST_TcPlcDeviceInput.uiCount

Use
Read the actual position.

iTcMc_EncoderM3120

The function block handles the evaluation of data from an M3120 incremental encoder box.

I/O variable
Value_N
State_N
Ctrl_N

iTcMc_EncoderSim

Interface.Variable
ST_TcPlcDeviceInput.uiCount
ST_TcPlcDeviceInput.usiStatus
ST_TcPlcDeviceOutput.usiCtrl

Use
Read the actual position.
Miscellaneous status information.
Control of the latch function etc.

A simulation encoder calculates the actual position through integration of the set velocity. No mapping is
required.

4.4.3.2

MC_AxRtReadForceDiff_BkPlcMc

Available from version 3.0

The function block handles determination of the actual force of the axis from the input data of two analog
input terminals. The actual pressure on the A- and B-sides is converted to the force acting on the load, taking
into account the areas and the sliding friction.

If only one input signal is available, a function block of type MC_AxRtReadForceSingle_BkPlcMc [} 218] should
be used. If the actual pressure is to be determined, a function block of type
MC_AxRtReadPressureDiff_BkPlcMc [} 220] should be used.

 Inputs
VAR_INPUT
    AdcValueA:      INT:=0;
    AdcValueB:      INT:=0;
    ScaleFactorA:   LREAL:=0.0;
    ScaleOffsetA:   LREAL:=0.0;
    ScaleFactorB:   LREAL:=0.0;
    ScaleOffsetB:   LREAL:=0.0;
    SlippingOffset: LREAL:=0.0;
    ReadingMode:    E_TcMcPressureReadingMode:=iTcHydPressureReadingDefault;
END_VAR

E_TcMcPressureReadingMode [} 119]

TF5810

Version: 1.8.3

215

MC_AxRtReadForceDiff_BkPlcMcAdcValueA  INTAdcValueB  INTScaleFactorA  LREALScaleOffsetA  LREALScaleFactorB  LREALScaleOffsetB  LREALSlippingOffset  LREALReadingMode  E_TcMcPressureReadingMode↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ErrorUDINT  ErrorIdPLCopen Motion Control

Name
AdcValueA
AdcValueB
ScaleFactorA

Type
INT
INT
LREAL

ScaleOffsetA

LREAL

ScaleFactorB

LREAL

ScaleOffsetB

LREAL

SlippingOffset

LREAL

ReadingMode

E_TcMcPressureReadin
gMode

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

 Outputs

VAR_OUTPUT
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
Error
ErrorID

Type
BOOL
UDINT

Behaviour of the function block:

Description
These parameters are used to transfer the input data of the
analog terminals.

[N/ADC_INC] This value represents the weighting. It
determines which pressure increase corresponds to a stage
of the AD converter.
[N/ADC_INC] This offset is used to correct the zero point of
the pressure scale.
[N/ADC_INC] This value represents the weighting. It
determines which pressure increase corresponds to a stage
of the AD converter.
[N/ADC_INC] This offset is used to correct the zero point of
the pressure scale.
[N] If the function block is used for calculating the active force,
the force required to overcome the sliding friction can be
entered here.
The actual value to be determined can be specified here.
AXIS_REF_BkPlcMc [} 86].ST_TcHydAxRtData
[} 141].fActPressure is selected as default target.

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc
[} 86] should be transferred.

Description
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is
provided here.

The function block determines the actual pressure and the actual force of the axis by evaluating the variables
AdcValueA and AdcValueB. The result is entered in ST_TcHydAxRtData [} 141].fActPressure.

The parameters assigned to an axis can be saved in ST_TcHydAxParam.fCustomerData[...], for
example. This ensures that the data are loaded, saved and backed up together with the standard
parameters of the axis and are also exported and imported, as required.

Determining a differential actual pressure

Commissioning is usually done in one of three ways.

Commissioning option A (preferred for ±10V)

In this case, no movement of the axis is required. The achievable accuracy is sufficient for high-quality
pressure sensors in most cases.

216

Version: 1.8.3

TF5810

PLCopen Motion Control

• The rated pressure of the pressure sensors divided by AdcValueAMAX or AdcValueBMAX should be

entered as ScaleFactorA and ScaleFactorB.

• If the function block is used for determining the actual pressure, the parameters ScaleArreaA and

ScaleArreaB should be set to 1.0. Otherwise these parameters should be specified for an actual force
in N (= Newton) in mm2.

Commissioning option B

For this option it is necessary that a function block can be approached with full system pressure in both
directions. A genuine movement of the axis is not required. Approaching of the end stops can be modeled by
limiting the axis movement through provisional limits or even complete mechanical fixing.

• All function blocks, which respond to the value of ST_TcHydAxRtData [} 141].fActPressure, must be

deactivated.

• First, slowly approach the lower function block (in the direction of decreasing actual position). The

values for AdcValueA and AdcValueB are determined and logged. The system pressure should now
be present on the A-side and the tank pressure – and therefore the ambient pressure – on the B-side.
Should this not be the case for some reason, the pressures on the A- and B-side should be determined
through measurement.

• Then, slowly approach the upper function block (in the direction of increasing actual position). The

values for AdcValueA and AdcValueB are again determined and logged. Now measure the pressures
again.

• The parameters to be entered can then be calculated as follows:

ScaleFactorA := (PressureAMAX - PressureAMIN) / (AdcValueAMAX - AdcValueAMIN);
ScaleFactorB := (PressureBMAX - PressureBMIN) / (AdcValueBMAX - AdcValueBMIN);
ScaleOffsetA := PressureAMIN - ScaleFactorA * AdcValueA;MIN
ScaleOffsetB := PressureBMIN - ScaleFactorB * AdcValueB;MIN

Commissioning option C

Alternatively, commissioning can be carried out without axis control. However, the accuracy that can be
achieved in this way is much lower.

• First, the axis should be made pressure-free. To this end, switch off the compressor and relieve the

pressure in the accumulator.

• Ensure that the axis does not build up pressure. To this end, an axis that is subject to external forces

(gravity etc.) should be supported mechanically. Open the valve several times in both directions, either
manually or electrically.

• Now determine and log the values for AdcValueA and AdcValueB. The tank pressure – and therefore
the ambient pressure – should be present both on the A-side and on the B-side. Should this not be the
case for some reason, the pressures on the A- and B-side should be determined through
measurement. Use the values found in this way as MIN values in the equations mentioned above.

• Take the pressure for the upper limit of the electrical signal (10 V, 20 mA) from the data sheet

specifications for the pressure sensors. Use the upper limit value for the converted electrical value as
AdcValueA and AdcValueB. Use these values as MAX values in the equations mentioned above.

• The parameters to be entered can then be calculated as described above.

Determining an active force

To determine an active force, first determine the actual pressure, as described above. Entering the active
areas under ScaleArreaA and ScaleArreaA causes the function block to convert the pressures on both
sides into forces, taking into account the areas.

TF5810

Version: 1.8.3

217

PLCopen Motion Control

4.4.3.3

MC_AxRtReadForceSingle_BkPlcMc

Available from version 3.0

The function block handles determination of the actual force of the axis from the input data of an analog input
terminal. The actual pressure on the A- or B-sides is converted to the force acting on the load, taking into
account the area and the sliding friction.

If only one input signal is available, a function block of type MC_AxRtReadForceDiff_BkPlcMc [} 215]
should be used. If the actual pressure is to be determined, a function block of type
MC_AxRtReadPressureDiff_BkPlcMc [} 220] should be used.

 Inputs
VAR_INPUT
    AdcValueA:      INT:=0;
    AdcValueB:      INT:=0;
    ScaleFactorA:   LREAL:=0.0;
    ScaleOffsetA:   LREAL:=0.0;
    ScaleFactorB:   LREAL:=0.0;
    ScaleOffsetB:   LREAL:=0.0;
    SlippingOffset: LREAL:=0.0;
    ReadingMode:    E_TcMcPressureReadingMode:=iTcHydPressureReadingDefault;
END_VAR

Name
AdcValueA
AdcValueB
ScaleFactorA

Type
INT
INT
LREAL

ScaleOffsetA

LREAL

ScaleFactorB

LREAL

ScaleOffsetB

LREAL

SlippingOffset

LREAL

ReadingMode

E_TcMcPressureReadingMo
de

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Description
These parameters are used to transfer the input data of
the analog terminals.

[N/ADC_INC] This value represents the weighting. It
determines which pressure increase corresponds to a
stage of the AD converter.
[N/ADC_INC] This offset is used to correct the zero point
of the pressure scale.
[N/ADC_INC] This value represents the weighting. It
determines which pressure increase corresponds to a
stage of the AD converter.
[N/ADC_INC] This offset is used to correct the zero point
of the pressure scale.
[N] If the function block is used for calculating the active
force, the force required to overcome the sliding friction
can be entered here.
The actual value to be determined can be specified here.
AXIS_REF_BkPlcMc [} 86].ST_TcHydAxRtData
[} 141].fActPressure is selected as default target.

218

Version: 1.8.3

TF5810

MC_AxRtReadForceSingle_BkPlcMcAdcValue  INTScaleFactor  LREALScaleOffset  LREALSlippingOffset  LREALReadingMode  E_TcMcPressureReadingMode↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ErrorUDINT  ErrorIdName
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

PLCopen Motion Control

 Outputs

VAR_OUTPUT
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
Error
ErrorID

Type
BOOL
UDINT

Description
Error: The occurrence of an error is indicated here.
ErrorID: Here, a coded cause of error is provided.

Behaviour of the function block:

The function block determines the actual pressure and the actual force of the axis by evaluating the variables
AdcValueA. The result is entered in ST_TcHydAxRtData [} 141].fActPressure.

The parameters assigned to an axis can be saved in ST_TcHydAxParam [} 130].fCustomerData[...], for
example. This ensures that the data are loaded, saved and backed up together with the standard parameters
of the axis and are also exported and imported, as required.

Determining a differential actual pressure

If the function block is used to determine the actual pressure, the parameters ScaleArreaA and
ScaleArreaA should be set to 1.0 and SlippingOffset to 0.0.

Commissioning option A

In this case, no movement of the axis is required. The achievable accuracy is sufficient for high-quality
pressure sensors in most cases.

• Enter the rated pressure of the pressure sensors divided by AdcValueAMAX as ScaleFactorA.

Commissioning option B

For this option it is necessary that a function block can be approached with full system pressure in both
directions. A genuine movement of the axis is not required. Approaching of the end stops can be modeled by
limiting the axis movement through provisional limits or even complete mechanical fixing.

• All function blocks, which respond to the value of ST_TcHydAxRtData [} 141].fActPressure, must be

deactivated.

• First, slowly approach the lower function block (in the direction of decreasing actual position). The

values for AdcValueA and AdcValueB are determined and logged. The system pressure should now
be present on the A-side and the tank pressure – and therefore the ambient pressure – on the B-side.
Should this not be the case for some reason, the pressures on the A- and B-side should be determined
through measurement.

• Then, slowly approach the upper function block (in the direction of increasing actual position). The

values for AdcValueA and AdcValueB are again determined and logged. Now measure the pressures
again.

• The parameters to be entered can then be calculated as follows:

ScaleFactorA := (PressureAMAX - PressureAMIN) / (AdcValueAMAX - AdcValueAMIN);
ScaleFactorB := (PressureBMAX - PressureBMIN) / (AdcValueBMAX - AdcValueBMIN);
ScaleOffsetA := PressureAMIN - ScaleFactorA * AdcValueA;MIN
ScaleOffsetB := PressureBMIN - ScaleFactorB * AdcValueB;MIN

TF5810

Version: 1.8.3

219

PLCopen Motion Control

Commissioning option C

Alternatively, commissioning can be carried out without axis control. However, the accuracy that can be
achieved in this way is much lower.

• First, the axis should be made pressure-free. To this end, switch off the compressor and relieve the

pressure in the accumulator.

• Ensure that the axis does not build up pressure. To this end, an axis that is subject to external forces

(gravity etc.) should be supported mechanically. Open the valve several times in both directions, either
manually or electrically.

• Now determine and log the values for AdcValueA and AdcValueB. The tank pressure – and therefore
the ambient pressure – should be present both on the A-side and on the B-side. Should this not be the
case for some reason, the pressures on the A- and B-side should be determined through
measurement. Use the values found in this way as MIN values in the equations mentioned above.

• Take the pressure for the upper limit of the electrical signal (10 V, 20 mA) from the data sheet

specifications for the pressure sensors. Use the upper limit value for the converted electrical value as
AdcValueA and AdcValueB. Use these values as MAX values in the equations mentioned above.

• The parameters to be entered can then be calculated as described above.

Determining an active force

To determine an active force, first determine the actual pressure, as described above. Entering the active
area under ScaleArreaA causes the function block to convert the single-sided pressure to a force, taking
into account the area.

4.4.3.4

MC_AxRtReadPressureDiff_BkPlcMc

Available from version 3.0

The function block handles determination of the actual pressure of the axis from the input data of two analog
input terminals.

If only one input signal is available, a function block of type MC_AxRtReadPressureSingle_BkPlcMc
[} 222] should be used. If the force is to be determined, instead of the pressure, a function block of
type MC_AxRtReadForceDiff_BkPlcMc [} 215] should be used.

 Inputs
VAR_INPUT
    AdcValueA:      INT:=0;
    AdcValueB:      INT:=0;
    ScaleFactorA:   LREAL:=0.0;
    ScaleOffsetA:   LREAL:=0.0;
    ScaleFactorB:   LREAL:=0.0;
    ScaleOffsetB:   LREAL:=0.0;
    ReadingMode:    E_TcMcPressureReadingMode:=iTcHydPressureReadingDefault;
END_VAR

220

Version: 1.8.3

TF5810

MC_AxRtReadPressureDiff_BkPlcMcAdcValueA  INTAdcValueB  INTScaleFactorA  LREALScaleOffsetA  LREALScaleFactorB  LREALScaleOffsetB  LREALReadingMode  E_TcMcPressureReadingMode↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ErrorUDINT  ErrorIdPLCopen Motion Control

Description
These parameters are used to transfer the input data of
the analog terminals.

[bar/ADC_INC] This value represents the weighting. It
determines which pressure increase corresponds to a
stage of the AD converter.
[bar] This offset is used to correct the zero point of the
pressure scale.
[bar/ADC_INC] This value represents the weighting. It
determines which pressure increase corresponds to a
stage of the AD converter.
[bar] This offset is used to correct the zero point of the
pressure scale.
This parameter is used to specify where the result of the
evaluation is to be stored.

Name
AdcValueA
AdcValueB
ScaleFactorA

Type
INT
INT
LREAL

ScaleOffsetA

LREAL

ScaleFactorB

LREAL

ScaleOffsetB

LREAL

ReadingMode

E_TcMcPressureReadingMo
de

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
Error
ErrorID

Type
BOOL
UDINT

Behaviour of the function block:

Description
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is
provided here.

The function block investigates the axis interface that has been passed to it every time it is called. During this
process, a problem may be detected and reported:

• If the pointer pStAxRtData in AXIS_REF_BkPlcMc [} 86] is not initialized, the function block reacts with
an Error and ErrorID:=dwTcHydErrCdPtrMcPlc. In this case, the axis cannot be placed into a fault
state.

If these checks could be performed without problem, the actual pressure of the axis is determined by
evaluating the variables AdcValueA and AdcValueB. The result is entered in ST_TcHydAxRtData
[} 141].fActPressure.

The parameters assigned to an axis can be saved in ST_TcHydAxParam [} 130].fCustomerData[...], for
example. This ensures that the data are loaded, saved and backed up together with the standard parameters
of the axis and are also exported and imported, as required.

Commissioning option A

In this case, no movement of the axis is required. The achievable accuracy is sufficient for high-quality
pressure sensors in most cases.

• The rated pressure of the pressure sensors divided by AdcValueAMAX or AdcValueBMAX should be

entered as ScaleFactorA and ScaleFactorB.

TF5810

Version: 1.8.3

221

PLCopen Motion Control

Commissioning option B

In this case, no movement of the axis is required. The achievable accuracy is sufficient for high-quality
pressure sensors in most cases.

• The rated pressure of the pressure sensors divided by AdcValueAMAX or AdcValueBMAX should be

entered as ScaleFactorA and ScaleFactorB.

Commissioning option C

For this option it is necessary that a function block can be approached with full system pressure in both
directions. A genuine movement of the axis is not required. Approaching of the end stops can be modeled by
limiting the axis movement through provisional limits or even complete mechanical fixing.

• All function blocks, which respond to the value of ST_TcHydAxRtData [} 141].fActPressure, must be

deactivated.

• First, slowly approach the lower function block (in the direction of decreasing actual position). The

values for AdcValueA and AdcValueB are determined and logged. The system pressure should now
be present on the B-side and the tank pressure – and therefore the ambient pressure – on the A-side.
Should this not be the case for some reason, the pressures on the A- and B-side should be determined
through measurement.

• Then, slowly approach the upper function block (in the direction of increasing actual position). The

values for AdcValueA and AdcValueB are again determined and logged. Now measure the pressures
again.

• The parameters to be entered can then be calculated as follows:

ScaleFactorA := (PressureAMAX - PressureAMIN) / (AdcValueAMAX - AdcValueAMIN);
ScaleFactorB := (PressureBMAX - PressureBMIN) / (AdcValueBMAX - AdcValueBMIN);
ScaleOffsetA := PressureAMIN - ScaleFactorA * AdcValueA;MIN
ScaleOffsetB := PressureBMIN - ScaleFactorB * AdcValueB;MIN

Commissioning option D

Alternatively, commissioning can be carried out without axis control. However, the accuracy that can be
achieved in this way is much lower.

• First, the axis should be made pressure-free. To this end, switch off the compressor and relieve the

pressure in the accumulator.

• Ensure that the axis does not build up pressure. To this end, an axis that is subject to external forces

(gravity etc.) should be supported mechanically. Open the valve several times in both directions, either
manually or electrically.

• Now determine and log the values for AdcValueA and AdcValueB. The tank pressure – and therefore
the ambient pressure – should be present both on the A-side and on the B-side. Should this not be the
case for some reason, the pressures on the A- and B-side should be determined through
measurement. Use the values found in this way as MIN values in the equations mentioned above.

• Take the pressure for the upper limit of the electrical signal (10 V, 20 mA) from the data sheet

specifications for the pressure sensors. Use the upper limit value for the converted electrical value as
AdcValueA and AdcValueB. Use these values as MAX values in the equations mentioned above.

• The parameters to be entered can then be calculated as described above.

4.4.3.5

MC_AxRtReadPressureSingle_BkPlcMc

222

Version: 1.8.3

TF5810

MC_AxRtReadPressureSingle_BkPlcMcAdcValue  INTScaleFactor  LREALScaleOffset  LREALReadingMode  E_TcMcPressureReadingMode↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ErrorUDINT  ErrorIdPLCopen Motion Control

Available from version 3.0

The function block handles determination of the actual pressure of the axis from the input data of an analog
input terminal.

If separate input signals are available for the A and B sides, a function block of type
MC_AxRtReadPressureDiff_BkPlcMc [} 220] should be used.

 Inputs
VAR_INPUT
    AdcValue:       INT:=0;
    ScaleFactor:    LREAL:=0.0;
    ScaleOffset:    LREAL:=0.0;
    ReadingMode:    E_TcMcPressureReadingMode:=iTcHydPressureReadingDefault;
END_VAR

Name
AdcValue

Type
INT

ScaleFactor

LREAL

ScaleOffset

LREAL

ReadingMode

E_TcMcPressureReadingMo
de

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Description
These parameters are used to transfer the input data of
the analog terminal.
[bar/ADC_INC] This value represents the weighting. It
determines which pressure increase corresponds to a
stage of the AD converter.
[bar] This offset is used to correct the zero point of the
pressure scale.
The actual value to be determined can be specified here.
AXIS_REF_BkPlcMc [} 86].ST_TcHydAxRtData
[} 141].fActPressure is selected as default value.

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
Error
ErrorID

Type
BOOL
UDINT

Description
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behaviour of the function block:

The function block investigates the axis interface that has been passed to it every time it is called. During this
process, a problem may be detected and reported:

• If the pointer pStAxRtData in AXIS_REF_BkPlcMc [} 86] is not initialised, the function block reacts with
an Error and ErrorID:=dwTcHydErrCdPtrMcPlc. In this case, the axis cannot be placed into a fault
state.

TF5810

Version: 1.8.3

223

PLCopen Motion Control

If these checks could be performed without problem, the actual pressure of the axis is determined by
evaluating the variables AdcValue. The result is entered in ST_TcHydAxRtData [} 141].fActPressure.

The parameters assigned to an axis can be saved in ST_TcHydAxParam [} 130].fCustomerData[...],
for example. This ensures that the data are loaded, saved and backed up together with the
standard parameters of the axis and are also exported and imported, as required.

Commissioning option A

For this option it is necessary that a function block can be approached with full system pressure in both
directions. A genuine movement of the axis is not required. Approaching of the end stops can be modeled by
limiting the axis movement through provisional limits or even complete mechanical fixing.

• All function blocks, which respond to the value of ST_TcHydAxRtData [} 141].fActPressure, must be

deactivated.

• First, slowly approach the lower function block (in the direction of decreasing actual position). The

value for AdcValue is determined and logged. The system pressure should now be present on the B-
side and the tank pressure – and therefore the ambient pressure – on the A-side. Should this not be
the case for some reason, the pressures on the A- and B-side should be determined through
measurement.

• Then, slowly approach the upper function block (in the direction of increasing actual position). The

value for AdcValue is determined and logged again. Now measure the pressures again.

• The parameters to be entered can then be calculated as follows:

ScaleFactor := (PressureMAX - PressureMIN) / (AdcValueMAX - AdcValueMIN);
ScaleOffset := PressureMIN - ScaleFactor * AdcValue;MIN

Commissioning option B

Alternatively, commissioning can be carried out without axis control. However, the accuracy that can be
achieved in this way is much lower.

• First, the axis should be made pressure-free. To this end, switch off the compressor and relieve the

pressure in the accumulator.

• Ensure that the axis does not build up pressure. To this end, an axis that is subject to external forces

(gravity etc.) should be supported mechanically. Open the valve several times in both directions, either
manually or electrically.

• Now the value for AdcValue is determined and logged. The tank pressure – and therefore the ambient
pressure – should be present both on the A-side and on the B-side. If this is not the case for some
reason, the pressure on the A-side should be determined through measurement. Use the values found
in this way as MIN values in the equations mentioned above.

• Take the pressure for the upper limit of the electrical signal (10 V, 20 mA) from the data sheet

specifications for the pressure sensors. Use the upper limit value for the converted electrical value as
AdcValue. Use these values as MAX values in the equations mentioned above.

• The parameters to be entered can then be calculated as described above.

4.4.3.6

MC_AxRtHybridAxisActuals_BkPlcMc

224

Version: 1.8.3

TF5810

MC_AxRtHybridAxisActuals_BkPlcMcstSystem_PrsIn  ST_TcPlcInputAnalogstAside_PrsIn  ST_TcPlcInputAnalogstBside_PrsIn  ST_TcPlcInputAnalognTorqueFeedback  INTudiMotorEnc_Count  UDINTbUpdateActForce  BOOLbPumpSwitch_Ext  BOOLbAreaSwitch_Ext  BOOL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ErrorUDINT  ErrorIdPLCopen Motion Control

Available from version 3.0.44

The function block determines the essential actual values of a servo-electric/hydraulic hybrid axis.

 Inputs
VAR_INPUT
    stSystem_PrsIn:    ST_TcPlcInputAnalog [} 164];
    stAside_PrsIn:     ST_TcPlcInputAnalog [} 164];
    stBside_PrsIn:     ST_TcPlcInputAnalog [} 164];

    nTorqueFeedback:   INT;
    udiMotorEnc_Count: UDINT;

    bUpdateActForce:   ST_ BOOL;

    bPumpSwitch_Ext:   BOOL;
    bAreaSwitch_Ext:   BOOL;

Name
stSystem_PrsIn

stAside_PrsIn

stBside_PrsIn

nTorqueFeedback

udiMotorEnc_Count

bUpdateActForce

bPumpSwitch_Ext

bAreaSwitch_Ext

Type
ST_TcPlcInputAnalog [} 164] If a pressure sensor is present at the pressurized

Description

reservoir, the input variables of the terminal are
transferred here.

ST_TcPlcInputAnalog [} 164] If a pressure sensor is present on the positive area

of the cylinder, the input variables of the terminal are
transferred here.

INT

UDINT

ST_TcPlcInputAnalog [} 164] If a pressure sensor is present on the negative area
of the cylinder, the input variables of the terminal are
transferred here.
The torque feedback signal of the drive is to be
transferred here.
The counter value of the motor encoder must be
transferred here.
With this signal, the function block calculates the
current actual force of the axis and updates it in
stAxRtData.fActForce.
This signal notifies the function block that pump
switching of the axis is active.
This signal notifies the function block that area
switching of the axis is active.

ST_ BOOL

BOOL

BOOL

 Inputs/outputs

VAR_IN_OUT
    Axis:   AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

 Outputs

VAR_OUTPUT
    Error:   BOOL;
    ErrorId: BOOL;
END_VAR

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86]
should be transferred.

TF5810

Version: 1.8.3

225

PLCopen Motion Control

Name
Error
ErrorId

Type
BOOL
BOOL

Description
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

If the axis is identified as 'hybrid', the function block performs the following steps.

• If a permissible encoder type for the motor encoder is specified for the axis, the function block

determines the current actual angle from udiMotorEnc_Count. Otherwise, the axis is set to a error
state and the angle is set to 0.0°.

• The modulo actual angle is updated with the set number of pump cavities.

• The actual pump speed is determined.

• The pump torque is determined.

• If a scaling pressure is set for the system pressure and the connection monitoring in stSystem_PrsIn

does not indicate a problem, the input value is converted to a pressure and updated in
stAxRtData.fSupplyPressure of the axis.

• If a scaling pressure is set for the pressure on the positive side and the connection monitoring in

stAside_PrsIn does not indicate a problem, the input value is converted to a pressure and updated in
stAxRtData.fActPressureA of the axis.

• If a scaling pressure is set for the pressure on the negative side and the connection monitoring in

stBside_PrsIn does not indicate a problem, the input value is converted to a pressure and updated in
stAxRtData.fActPressureB of the axis.

• If TRUE is passed in bUpdateActForce, the function block stAxRtData.fActForce updates the axis,

using the currently effective areas.

• If an edge is detected at one of the switch signals (bAreaSwitch_Ext, bPumpSwitch_Ext), the

function block initiates a ramp for changing the feed constant and the maximum speed.

Irrespective of whether the axis is identified as 'hybrid', this function block calls a local instance of
MC_AxRtEncoder_BkPlcMc() [} 198] for the axis.

If no function block of this type is called for a servo-electric/hydraulic axis, changeovers are not
handled correctly. This could lead to unexpected behavior of the axis. In this case, the axis is set to
the error state and a message is output.

4.4.4

FunctionGenerator

4.4.4.1

MC_FunctionGeneratorFD_BkPlcMc

Available from version 3.0.31

The function block calculates the signals of a function generator.

 Inputs/outputs

VAR_IN_OUT
    stTimeBase:     ST_FunctionGeneratorTB_BkPlcMc;
    stFunctionDef:  ST_FunctionGeneratorFD_BkPlcMc;
END_VAR

226

Version: 1.8.3

TF5810

MC_FunctionGeneratorFD_BkPlcMc↔stTimeBase  Reference To ST_FunctionGeneratorTB_BkPlcMc↔stFunctionDef  Reference To ST_FunctionGeneratorFD_BkPlcMcLREAL  SinusLREAL  CosinusLREAL  RectangleLREAL  SawToothName
stTimeBase

stFunctionDef

Type
ST_FunctionGeneratorTB_Bk
PlcMc
ST_FunctionGeneratorFD_Bk
PlcMc

Description
stTimeBase: A structure with the parameters of the time
base of this function generator.
stFunctionDef: A structure with the definitions of the
output signals of a function generator.

PLCopen Motion Control

 Outputs

VAR_OUTPUT
    Sinus:          LREAL;
    Cosinus:        LREAL;
    Rectangle:      LREAL;
    SawTooth:       LREAL;
END_VAR

Description
The output signals of the function generator.

Name
Sine
Cosine
Rectangle
SawTooth

Type
LREAL
LREAL
LREAL
LREAL

Behavior of the function block

The output signals are determined from stTimeBase.CurrentRatio and the parameters in stFunctionDef
[} 127].

The time base in stTimeBase should be updated with an MC_FunctionGeneratorTB_BkPlcMc [} 228]()
function block.

To change the operating frequency, an MC_FunctionGeneratorSetFrq_BkPlcMc [} 227]() function block should
be used.

4.4.4.2

MC_FunctionGeneratorSetFrq_BkPlcMc

Available from version 3.0.31

The function block updates the operating frequency of a time base for one or several function generators
[} 226].

 Inputs
VAR_INPUT
    Frequency:      LREAL;
    CycleTime:      LREAL;
END_VAR

Name
Frequency
CycleTime

Type
LREAL
LREAL

Description
The operating frequency to be used.
The cycle time of the calling task.

TF5810

Version: 1.8.3

227

MC_FunctionGeneratorSetFrq_BkPlcMcFrequency  LREALCycleTime  LREAL↔stTimeBase  Reference To ST_FunctionGeneratorTB_BkPlcMcPLCopen Motion Control

 Inputs/outputs

VAR_IN_OUT
    stTimeBase:     ST_FunctionGeneratorTB_BkPlcMc;
END_VAR

Name
stTimeBase

Type
ST_FunctionGeneratorTB_Bk
PlcMc

Description
A structure with the parameters of the time base of one or
several function generators [} 128].

Behavior of the function block

The function block sets stTimeBase.Frequency to the transferred value. stTimeBase.CurrentTime is
adjusted, if required.

The function block uses stTimeBase.Freeze to prevent a collision with MC_FunctionGeneratorTB_BkPlcMc
[} 228]() function blocks. Thus, it can also be called from another task.

4.4.4.3

MC_FunctionGeneratorTB_BkPlcMc

Available from version 3.0.31

The function block updates a time base for one or several function generators [} 226].

 Inputs
VAR_INPUT
    CycleTime:      LREAL;
END_VAR

Name
CycleTime

Type
LREAL

Description
The cycle time of the calling task.

 Inputs/outputs

VAR_IN_OUT
    stTimeBase:     ST_FunctionGeneratorTB_BkPlcMc;
END_VAR

Name
stTimeBase

Type
ST_FunctionGeneratorTB_BkPl
cMc

Description
A structure with the parameters of the time base of one or
several function generators [} 128].

Behavior of the function block

If stTimeBase.Freeze is not set, stTimeBase.CurrentTime is updated with CycleTime and
stTimeBase.CurrentRatio is determined. stTimeBase.Frequency is taken into account.

To change the operating frequency, an MC_FunctionGeneratorSetFrq_BkPlcMc [} 227]() function block should
be used.

228

Version: 1.8.3

TF5810

MC_FunctionGeneratorTB_BkPlcMcCycleTime  LREAL↔stTimeBase  Reference To ST_FunctionGeneratorTB_BkPlcMc4.4.5

TableFunctions

4.4.5.1

MC_AxTableFromAsciFile_BkPlcMc

PLCopen Motion Control

Available from version 3.0

The function block reads the contents of a table from a text file.

 Inputs
VAR_INPUT
    Execute:    BOOL:=FALSE;
    pTable:     POINTER TO LREAL:=0;
    LowIdx:     INT:=0;
    HighIdx:    INT:=0;
    FileName:   STRING(255):='';
END_VAR

Name
Execute
pTable

LowIdx

HighIdx

Type
BOOL
POINTER TO LREAL

INT

INT

FileName

STRING

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Description
A rising edge at this input starts the read process.
This parameter is used to transfer the address of an
ARRAY[nFirstIdx..nLastIdx,1..2].
This parameter is used to transfer the lower index of the ARRAY,
whose address is transferred as pTable.
This parameter is used to transfer the upper index of the ARRAY,
whose address is transferred as pTable.
This parameter can be used to specify a file name.

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:       BOOL;
    Done:       BOOL;
    Error:      BOOL;
    ErrorID:    UDINT;
    LastIdx:    INT:=0;
END_VAR

TF5810

Version: 1.8.3

229

PLCopen Motion Control

Name
Busy
Done
Error
ErrorID
LastIdx

Type
BOOL
BOOL
BOOL
UDINT
INT

Description
Indicates that a command is being processed.
Successful processing of the homing is indicated here.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.
This parameter is used to indicate the index of the last table row defined by the read
operation.

Behavior of the function block

A rising edge at Execute causes the function block to check the transferred parameters. A number of
problems can be detected and reported during this process:

• If LowIdx is negative the system responds with Error and ErrorID=dwTcHydErrCdTblEntryCount.

• If pTable=0 the system responds with Error and ErrorID=dwTcHydErrCdTblEntryCount.

• If LowIdx and HighIdx describe a table with less than five rows the system responds with Error and

ErrorID=dwTcHydErrCdTblEntryCount.

If these checks were performed without problems, the read operation is started. Busy is TRUE for the
duration of the operation. This can lead to some further problems, which are indicated by various error
codes. Successful reading of the file is indicated with Done.

A falling edge at Execute clears all the pending output signals. If Execute is set to FALSE while the process
is still active, the process that had started continues unaffected. The signals at the end of the process (Error,
ErrorID, Done) are made available for one cycle.

If a FileName is specified, it must be complete (including the drive letter and the path, if applicable, always
including the file type), since it is used by function block without any further modification or amendment.

If no FileName is specified, the function block uses the path and the file name, which were specified through
the MC_AxUtiStandardInit_BkPlcMc [} 254] function block. File type TXT is used here, to distinguish from the
parameter file with file type DAT.

The file contents can be read or modified with an ASCII editor. Changes of the content can make
correct reading or the intended use impossible or change the effect of the table in a way that is
difficult to trace. Manual changes should therefore be implemented very carefully, if at all, and only
by competent persons.

4.4.5.2

MC_AxTableFromBinFile_BkPlcMc

Available from version 3.0

The function block reads the contents of a table from a binary file.

230

Version: 1.8.3

TF5810

PLCopen Motion Control

Description
A rising edge at this input starts the read process.
This parameter is used to transfer the address of an
ARRAY[nFirstIdx..nLastIdx,1..2].
This parameter is used to transfer the lower index of the ARRAY,
whose address is transferred as pTable.
This parameter is used to transfer the upper index of the ARRAY,
whose address is transferred as pTable.
This parameter can be used to specify a file name.

 Inputs
VAR_INPUT
    Execute:    BOOL:=FALSE;
    pTable:     POINTER TO LREAL:=0;
    LowIdx:     INT:=0;
    HighIdx:    INT:=0;
    FileName:   STRING(255):='';
END_VAR

Name
Execute
pTable

LowIdx

HighIdx

Type
BOOL
POINTER TO LREAL

INT

INT

FileName

STRING

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:       BOOL;
    Done:       BOOL;
    Error:      BOOL;
    ErrorID:    UDINT;
    LastIdx:    INT:=0;
END_VAR

Name
Busy
Done
Error
ErrorID
LastIdx

Type
BOOL
BOOL
BOOL
UDINT
INT

Description
Indicates that a command is being processed.
Successful processing of the homing is indicated here.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.
This parameter is used to indicate the index of the last table row defined by the read
operation.

Behavior of the function block

A rising edge at Execute causes the function block to check the transferred parameters. A number of
problems can be detected and reported during this process:

• If LowIdx is negative the system responds with Error and ErrorID=dwTcHydErrCdTblEntryCount.

• If pTable=0 the system responds with Error and ErrorID=dwTcHydErrCdTblEntryCount.

• If LowIdx and HighIdx describe a table with less than five rows the system responds with Error and

ErrorID=dwTcHydErrCdTblEntryCount.

If these checks were performed without problems, the read operation is started. Busy is TRUE for the
duration of the operation. This can lead to some further problems, which are indicated by various error
codes. Successful reading of the file is indicated with Done.

TF5810

Version: 1.8.3

231

PLCopen Motion Control

A falling edge at Execute clears all the pending output signals. If Execute is set to FALSE while the process
is still active, the process that had started continues unaffected. The signals at the end of the process (Error,
ErrorID, Done) are made available for one cycle.

If a FileName is specified, it must be complete (including the drive letter and the path, if applicable, always
including the file type), since it is used by function block without any further modification or amendment.

If no FileName is specified, the function block uses the path and the file name, which were specified through
the MC_AxUtiStandardInit_BkPlcMc [} 254] function block. File type TBL is used here, to distinguish from the
parameter file with file type DAT.

The file contents cannot be read or modified with an ASCII editor.

4.4.5.3

MC_AxTableReadOutNonCyclic_BkPlcMc

Available from version 3.0

The function block determines the slave values assigned to a master value with the aid of a table.

This function block is a component of cam plates or similar non-linear couplings. It is generally not
called direct by an application.

 Inputs
VAR_INPUT
    pTable:         POINTER TO LREAL:=0;
    fMasterValue:   LREAL:=0.0;
    nFirstIdx:      UDINT:=1;
    nLastIdx:       UDINT:=1;
    bReInit:        BOOL:=FALSE;
END_VAR

232

Version: 1.8.3

TF5810

MC_AxTableReadOutNonCyclic_BkPlcMcpTable  Pointer To LREALfMasterValue  LREALnFirstIdx  UDINTnLastIdx  UDINTbReInit  BOOLLREAL  fSlaveValueLREAL  fSlaveGearBOOL  bUnderRangeBOOL  bOverRangeBOOL  bErrorName
pTable

Type
POINTER TO
LREAL

Description
This parameter is used to transfer the address of an
ARRAY[nFirstIdx..nLastIdx,1..2].

PLCopen Motion Control

An incorrect specification at this point causes the PLC application
to crash by triggering serious runtime errors (Page Fault
Exception).
Here the value of the master is to be transferred, for which the
associated slave
This parameter is used to transfer the lower index of the ARRAY, whose
address is transferred as pTable.

An incorrect specification at this point causes the PLC application
to crash by triggering serious runtime errors (Page Fault
Exception).
This parameter is used to transfer the upper index of the ARRAY, whose
address is transferred as pTable.

An incorrect specification at this point causes the PLC application
to crash by triggering serious runtime errors (Page Fault
Exception).
This input indicates to the function block that the search procedure
should start at the top of the table.

fMasterValue

LREAL

nFirstIdx

UDINT

nLastIdx

UDINT

bReInit

BOOL

 Outputs

VAR_OUTPUT
    fSlaveValue:    LREAL:=0.0;
    fSlaveGear:     LREAL:=0.0;
    bUnderRange:    BOOL;
    bOverRange:     BOOL;
END_VAR

Name
fSlaveValue
fSlaveGear

Type
LREAL
LREAL

bUnderRange

BOOL

bOverRange

BOOL

Description
This parameter is used to output the slave value belonging to fMasterValue.
This parameter is used to output the local slope of the slave values at the
point in the table specified by the master.
This output becomes TRUE, if the master value reaches the bottom of the
table or falls below it.
This output becomes TRUE, if the master value reaches the top of the table
or exceeds it.

Behavior of the function block

The function block searches inside the transferred table for a master pair of values, which matches or
includes the transferred fMasterValue. Within the found intervals a linear intermediate interpolation is
calculated. The result is output as fSlaveValue. The local slope determined in this calculation is output as
fSlaveGear.

If fMasterValue is below the value range described by the table, bUnderRange is indicated. The value
output as fSlaveValue is the value assigned to the lowest point of the table. 0.0 is returned as fSlaveGear.

If the fMasterValue is above the range of values described by the table, bOverRange is indicated. The
value output as fSlaveValue is the value assigned to the top point of the table. 0.0 is returned as
fSlaveGear.

The return value fSlaveGear represents the ratio of the first derivatives of fMasterValue and fSlaveValue. If
fMasterValue represents a position or a virtual time, the multiplication of master progress velocity and
fSlaveGear returns the set slave velocity. This can be used to generate a pilot-control velocity. An
MC_AxRtSetExtGenValues_BkPlcMc [} 252] function block is preferable for this purpose.

TF5810

Version: 1.8.3

233

PLCopen Motion Control

4.4.5.4

MC_AxTableToAsciFile_BkPlcMc

Available from version 3.0

The function block writes the contents of a table to a text file.

 Inputs
VAR_INPUT
    Execute:    BOOL:=FALSE;
    pTable:     POINTER TO LREAL:=0;
    LowIdx:     INT:=0;
    HighIdx:    INT:=0;
    FileName:   STRING(255):='';
END_VAR

Name
Execute
pTable

LowIdx

HighIdx

Type
BOOL
POINTER TO LREAL

INT

INT

FileName

STRING

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

 Outputs

VAR_OUTPUT
    Busy:       BOOL;
    Done:       BOOL;
    Error:      BOOL;
    ErrorID:    UDINT;
END_VAR

Description
The writing process is initiated by a rising edge at this input.
This parameter is used to transfer the address of an
ARRAY[nFirstIdx..nLastIdx,1..2].
This parameter is used to transfer the lower index of the ARRAY,
whose address is transferred as pTable.
This parameter is used to transfer the upper index of the ARRAY,
whose address is transferred as pTable.
This parameter can be used to specify a file name.

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc
[} 86] should be transferred.

Name
Busy
Done
Error
ErrorID

Type
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
Successful processing of the homing is indicated here.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

234

Version: 1.8.3

TF5810

MC_AxTableReadOutNonCyclic_BkPlcMcpTable  Pointer To LREALfMasterValue  LREALnFirstIdx  UDINTnLastIdx  UDINTbReInit  BOOLLREAL  fSlaveValueLREAL  fSlaveGearBOOL  bUnderRangeBOOL  bOverRangeBOOL  bErrorPLCopen Motion Control

Behavior of the function block

A rising edge at Execute causes the function block to check the transferred parameters. A number of
problems can be detected and reported during this process:

• If LowIdx is negative the system responds with Error and ErrorID=dwTcHydErrCdTblEntryCount.

• If pTable=0 the system responds with Error and ErrorID=dwTcHydErrCdTblEntryCount.

• If LowIdx and HighIdx describe a table with less than five rows the system responds with Error and

ErrorID=dwTcHydErrCdTblEntryCount.

If these checks were performed without problems, the write operation is started. Busy is TRUE for the
duration of the operation. This can lead to some further problems, which are indicated by various error
codes. Successful writing of the file is indicated with Done.

A falling edge at Execute clears all the pending output signals. If Execute is set to FALSE while the process
is still active, the process that had started continues unaffected. The signals at the end of the process (Error,
ErrorID, Done) are made available for one cycle.

If a FileName is specified, it must be complete (including the drive letter and the path, if applicable, always
including the file type), since it is used by function block without any further modification or amendment.

If no FileName is specified, the function block uses the path and the file name, which were specified through
the MC_AxUtiStandardInit_BkPlcMc [} 254] function block. File type TXT is used here, to distinguish from the
parameter file with file type DAT.

The file contents can be read or modified with an ASCII editor. Changes of the content can make
correct reading or the intended use impossible or change the effect of the table in a way that is
difficult to trace. Manual changes should therefore be implemented very carefully, if at all, and only
by competent persons.

4.4.5.5

MC_AxTableToBinFile_BkPlcMc

Available from version 3.0

The function block writes the contents of a table to a binary file.

 Inputs
VAR_INPUT
    Execute:    BOOL:=FALSE;
    pTable:     POINTER TO LREAL:=0;
    LowIdx:     INT:=0;
    HighIdx:    INT:=0;
    FileName:   STRING(255):='';
END_VAR

TF5810

Version: 1.8.3

235

PLCopen Motion Control

Name
Execute
pTable

LowIdx

HighIdx

Type
BOOL
POINTER TO LREAL

INT

INT

FileName

STRING

Description
The writing process is initiated by a rising edge at this input.
This parameter is used to transfer the address of an
ARRAY[nFirstIdx..nLastIdx,1..2].
This parameter is used to transfer the lower index of the ARRAY,
whose address is transferred as pTable.
This parameter is used to transfer the upper index of the ARRAY,
whose address is transferred as pTable.
This parameter can be used to specify a file name.

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:       BOOL;
    Done:       BOOL;
    Error:      BOOL;
    ErrorID:    UDINT;
END_VAR

Name
Busy
Done
Error
ErrorID

Type
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
Successful processing of the homing is indicated here.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

A rising edge at Execute causes the function block to check the transferred parameters. A number of
problems can be detected and reported during this process:

• If LowIdx is negative the system responds with Error and ErrorID=dwTcHydErrCdTblEntryCount.

• If pTable=0 the system responds with Error and ErrorID=dwTcHydErrCdTblEntryCount.

• If LowIdx and HighIdx describe a table with less than five rows the system responds with Error and

ErrorID=dwTcHydErrCdTblEntryCount.

If these checks were performed without problems, the write operation is started. Busy is TRUE for the
duration of the operation. This can lead to some further problems, which are indicated by various error
codes. Successful writing of the file is indicated with Done.

A falling edge at Execute clears all the pending output signals. If Execute is set to FALSE while the process
is still active, the process that had started continues unaffected. The signals at the end of the process (Error,
ErrorID, Done) are made available for one cycle.

If a FileName is specified, it must be complete (including the drive letter and the path, if applicable, always
including the file type), since it is used by function block without any further modification or amendment.

If no FileName is specified, the function block uses the path and the file name, which were specified through
the MC_AxUtiStandardInit_BkPlcMc [} 254] function block. File type TBL is used here, to distinguish from the
parameter file with file type DAT.

236

Version: 1.8.3

TF5810

PLCopen Motion Control

The file contents cannot be read or modified with an ASCII editor.

4.4.6

Generators

4.4.6.1

MC_AxRtGenerator_BkPlcMc

This function block performs the task of a setpoint generator. To this end a profile-specific function block is
called, depending on the value set as nProfileType in Axis.ST_TcHydAxParam [} 130].

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Error:      BOOL;
    ErrorID:    UDINT;
    LagError:   BOOL;
END_VAR

Name
Error
ErrorID
LagError

Type
BOOL
UDINT
BOOL

Description
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.
If the lag error exceeds the set limits, it is indicated here. This signal is also
available if position lag monitoring is not activated.

Behavior of the function block

The function block investigates the axis interface that has been passed to it every time it is called. A number
of problems can be detected and reported during this process:

• If one of the pointers has not been initialized the function block reacts with Error and with

ErrorID:=dwTcHydErrCdPtrPlcMc or dwTcHydErrCdPtrMcPlc.

If it is possible to carry out these checks without encountering any problems, the setpoint generation is
executed by calling an appropriate function block corresponding to the nProfileType in
Axis.ST_TcHydAxParam [} 130].

The LagError output indicates whether the current lag error of the axis exceeds the set limits. The axis is
only set to an error state if bMaxLagEna is set in Axis.ST_TcHydAxParam [} 130].

The following generators are presently available:

TF5810

Version: 1.8.3

237

MC_AxRtGenerator_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ErrorUDINT  ErrorIDBOOL  LagErrorPLCopen Motion Control

nProfileType
iTcMc_ProfileCtrlBased [} 239] Standard profile: Single-stage time-referenced acceleration, displacement-

Description

referenced (square root) braking ramp, target approach at creep velocity,
selectable behavior when stationary.

iTcMc_ProfileJerkBased

iTcMc_ProfileTimePosCtrl

iTcMc_ProfileCosine

iTcMc_ProfileTimeRamp
[} 241]

An axis in motion can be restarted at any time (new target, new velocity etc.),
except in error state or in a state with dependent control value generation.

Info: Overshooting the new target can happen even if the axis is in front of
the target position at the time of the start.

Info: The function block can be parameterized such that it starts
automatically and assumes an active motion state under certain conditions,
which are defined through its parameters.

Info: This generator type can optionally operate in purely time-controlled
mode with continuously closed position controller.
Standard profile: Single- or two-stage time-controlled acceleration through
optional jerk limitation, displacement-controlled (square root generator)
braking ramp, target approach with jerk limitation, selectable behavior in idle
state.

An axis in motion can be restarted at any time (new target, new velocity etc.),
except in error state or in a state with dependent control value generation.

Info: Overshooting the new target can happen even if the axis is in front of
the target position at the time of the start.

Info: The function block can be parameterized such that it starts
automatically and assumes an active motion state under certain conditions,
which are defined through its parameters.

Info: This generator type can optionally operate in purely time-controlled
mode with continuously closed position controller.

Info: Some functions are not supported by this generator type, or not fully.
Info: Only present for compatibility reasons; will shortly no longer be
supported.

Special profile: Two stage acceleration (initially time-referenced, then
displacement-referenced following square root curve), displacement-
referenced (square root) braking ramp, target approach at creep velocity,
selectable behavior when stationary.

It is not possible to execute a start for an axis that is already travelling (new
target, new velocity etc.).
Info: Only present for compatibility reasons; will shortly no longer be
supported.

Special profile: Two stage acceleration (initially time-referenced, then
displacement-referenced following cosine curve), displacement-referenced
(cosine) braking ramp, target approach at creep velocity, selectable behavior
when stationary.

It is not possible to execute a start for an axis that is already travelling (new
target, new velocity etc.).
Special profile: Single-stage time-controlled acceleration, time-controlled
braking ramp, target approach with creep speed, conditionally selectable
behavior in idle state. The generator uses position cams instead of an
encoder.

An axis in motion can be restarted (new target, new velocity etc.), except in
error state.

Info: This generator type is intended for axes, which only have digital cams
instead of an encoder.

If only the usual function blocks (encoder, generator, finish, drive) for the axis are to be called, a function
block of type MC_AxStandardBody_BkPlcMc [} 253] should be used for simplicity.

238

Version: 1.8.3

TF5810

iTcMc_ProfileCtrlBased

A profile is generated with a time-controlled acceleration phase, a displacement-controlled braking phase
based on the square root generator principle, and a target approach with creep speed.

PLCopen Motion Control

The arrows on the profile of the control value suggest how the shape of the curve can be affected through
the parameters of the move order or of the axis. To begin with, a time-controlled ramp function "1" is used to
accelerate to the required travel velocity "2". This control value is maintained until a point is reached that was
recalculated at the start. After this point, a displacement-referenced ramp "3" is followed to brake down from
the main travel velocity to the creep velocity "5"; this control value is reached at a specified distance, "4",
from the target. This control value is retained until the target has been approached to within a specified
remaining distance "6". The axis is then switched to its idle behavior.

Parameters active in the travel profile

Start ramp "1": The smallest of the following values is the effective one: fMaxAcc and fAcc in
Axis.ST_TcHydAxParam [} 130], Acceleration of the function block used to start the axis (for example:
MC_MoveAbsolute_BkPlcMc [} 73]).

Travel phase "2": The smallest of the following values is the effective one: fRefVelo and fMaxVelo in
Axis.ST_TcHydAxParam [} 130], Velocity of the function block used to start the axis (for example:
MC_MoveAbsolute_BkPlcMc [} 73]).

Braking ramp "3": The smallest of the following values is the effective one: fMaxDec and fDec in
Axis.ST_TcHydAxParam [} 130], Deceleration of the function block used to start the axis (for example:
MC_MoveAbsolute_BkPlcMc [} 73]).

Creep phase "4", "5": The values of fCreepSpeed and fCreepDistance in Axis.ST_TcHydAxParam [} 130]
have an effect.

TF5810

Version: 1.8.3

239

PLCopen Motion Control

Transfer to target "6": The fBrakeDistance and/or fBrakeDeadTime in Axis.ST_TcHydAxParam [} 130]
have an effect.

Automatic starting of the axis

If the difference between the actual position and the current target position exceeds the value in
Axis.ST_TcHydAxParam [} 130].fReposDistance, an automatic start is triggered.

iTcMc_ProfileJerkBased

A profile is generated with a time-controlled acceleration phase (with optional jerk limitation), a displacement-
controlled braking ramp based on the square root generator principle, and a target approach with jerk
limitation.

The arrows on the profile of the control value suggest how the shape of the curve can be affected through
the parameters of the move order or of the axis. To begin with, a time-controlled ramp function "1" is used to
accelerate to the required travel velocity "2". The optional jerk limitation "6" can take effect. The travel speed
is maintained until a point is reached that was recalculated at the start. At this point a displacement-
controlled braking ramp "3" is applied, until the distance to the target has reduced to the residual distance.
The deceleration "4" is reduced with limited jerk "5" towards the target. The axis is then switched to its idle
behavior.

Parameters active in the travel profile

Start ramp "1": The smallest of the following values is the effective one: fMaxAcc and fAcc in
Axis.ST_TcHydAxParam [} 130], Acceleration of the function block used to start the axis (for example:
MC_MoveAbsolute_BkPlcMc [} 73]).

240

Version: 1.8.3

TF5810

PLCopen Motion Control

Travel phase "2": The smallest of the following values is the effective one: fRefVelo and fMaxVelo in
Axis.ST_TcHydAxParam [} 130], Velocity of the function block used to start the axis (for example:
MC_MoveAbsolute_BkPlcMc [} 73]).

Braking ramp "3", "4": The smallest of the following values is the effective one: fMaxDec and fDec in
Axis.ST_TcHydAxParam [} 130], Deceleration of the function block used to start the axis (for example:
MC_MoveAbsolute_BkPlcMc [} 73]).

Transfer to target "5": fMaxJerk in Axis.ST_TcHydAxParam [} 130] and fJerk of the function block used on
axis start take effect (example: MC_MoveAbsolute_BkPlcMc [} 73]) and fBrakeDistance and/or
fBrakeDeadTime in Axis.ST_TcHydAxParam [} 130].

iTcMc_ProfileTimePosCtrl

Only present for compatibility reasons; will shortly no longer be supported. It should not be used for
new projects and should be replaced when existing projects are revised, if possible.

iTcMc_ProfileCosine

Only present for compatibility reasons; will shortly no longer be supported. It should not be used for
new projects and should be replaced when existing projects are revised, if possible.

iTcMc_ProfileTimeRamp

A profile is generated with a time-controlled acceleration phase, a time-controlled braking phase and a target
approach with creep speed.

TF5810

Version: 1.8.3

241

PLCopen Motion Control

The arrows on the profile of the control value suggest how the shape of the curve can be affected through
the parameters of the move order or of the axis. To begin with, a time-controlled ramp function "1" is used to
accelerate to the required travel velocity "2". This control value is maintained until the direction-specific target
window cam is detected. From here, a time-controlled ramp "3" is applied to decelerate from the set motion
value to the set creep value "5". This control value is maintained until the direction-specific target cam is
detected. The axis is then switched to its idle behavior.

Parameters active in the travel profile

Start ramp "1": fStartRamp has an effect in Axis.ST_TcHydAxParam [} 130].

Travel phase "2": The smallest of the following values is the effective one: fRefVelo and fMaxVelo in
Axis.ST_TcHydAxParam [} 130], Velocity of the function block used to start the axis (for example:
MC_MoveAbsolute_BkPlcMc [} 73]).

Braking ramp "3": fStopRamp has an effect in Axis.ST_TcHydAxParam [} 130].

Creep phase "4": fCreepSpeed has an effect in Axis.ST_TcHydAxParam [} 130].

Behavior of the function block on restart during a motion

If a further start command is issued during an active movement, a distinction has to be made between two
cases.

This profile is created on restart in the same direction with a different velocity (higher in this case).

242

Version: 1.8.3

TF5810

PLCopen Motion Control

This profile is created on restart in the opposite direction, in this case with the same velocity.

This profile type can only be used in a meaningful manner in combination with the encoder type
iTcMc_EncoderDigCam [} 207]. See also Special case: digital position cams.

4.4.6.2

MC_AxRuntime_BkPlcMc

Available from version 3.0

The function block integrates a function block of the type MC_AxRtGenerator_BkPlcMc() [} 237] and a
function block of the type MC_AxRtController_BkPlcMc() [} 245]. The outputs of the generator are forwarded.

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

TF5810

Version: 1.8.3

243

MC_AxRuntime_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ErrorUDINT  ErrorIDBOOL  LagErrorPLCopen Motion Control

 Outputs

VAR_OUTPUT
    Error:      BOOL;
    ErrorID:    UDINT;
    LagError:   BOOL;
END_VAR

Name
Error
ErrorID
LagError

Type
BOOL
UDINT
BOOL

Description
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.
If the lag error exceeds the set limits, it is indicated here. This signal is also
available if position lag monitoring is not activated.

4.4.7

Runtime

4.4.7.1

MC_AxRtCheckSyncDistance_BkPlcMc

Available from version 3.0

The function block checks for an invalid path (distance) after leaving the cam during homing.

 Inputs
VAR_INPUT
    MaxDistance:    LREAL;
    MinDistance:    LREAL;
    MaxIndexWidth:  LREAL;
END_VAR

Name
MaxDistance

Type
LREAL

MinDistance

LREAL

MaxIndexWidth

LREAL

Description
[mm] This parameter is used to specify the maximum permitted distance
that may be traveled between the referencing cam and reaching of the zero
pulse.
[mm] This parameter is used to specify the minimum distance that must be
traveled between the referencing cam and reaching of the zero pulse.
[mm] This parameter is used to specify the minimum distance that must be
traveled to leave the referencing cam. (from V3.0.20)

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

244

Version: 1.8.3

TF5810

MC_AxRtCheckSyncDistance_BkPlcMcMaxDistance  LREALMinDistance  LREALMaxIndexWidth  LREAL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ActiveBOOL  ExceededPLCopen Motion Control

 Outputs

VAR_OUTPUT
    Active:         BOOL;
    Exceeded:       BOOL;
END_VAR

Name
Active

Type
BOOL

Exceeded

BOOL

Description
Indicates that the axis has left the cam has and expects the zero pulse of the
encoder.
Indicates that the axis has travelled more than MaxDistance after leaving the cam,
without detection of the zero pulse of the encoder.

Behavior of the function block

The function block detects the part of the homing, in which the axis searches for the zero pulse of encoder,
thereby monitoring the distance travelled. Two problems can be detected during this process:

• The axis travels MaxIndexWidth, without that the falling edges of the referencing cam being detected.

• The axis travels MaxDistance, without a zero pulse being detected.

• The zero pulse is detected, before the axis has traveled MinDistance.

Any problems that are detected are indicated with Exceeded. If this is to lead to an axis error, the application
must specify a corresponding change of state. An MC_AxRtGoErrorState_BkPlcMc [} 249] function block and
a coded Error Code [} 339] should be used here.

Monitoring for MinDistance and MaxDistance can be suppressed by setting the respective
parameter to 0.0.

4.4.7.2

MC_AxRtController_BkPlcMc

This function block contains the standard position controller of the axis.

If necessary, a function block of the significantly more complex type MC_AxRtPosPiControllerEx_BkPlcMc()
[} 185] can be used instead of this function block.

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here a variable of the type AXIS_REF_BkPlcMc [} 86] should be
transferred.

TF5810

Version: 1.8.3

245

MC_AxRtController_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcPLCopen Motion Control

Structure of the controller

The Ti parameter is used by this controller as Ki. A value of 0.0 disables the I part. Increasing
values generate increasingly strong reactions of the I part.

4.4.7.3

MC_AxRtFinish_BkPlcMc

Available from version 3.0

This function block adapts the control value that has been generated to the special features of the particular
axis. An MC_AxRtFinishLinear_BkPlcMc [} 247] function block should be used if a characteristic curve
linearization is required.

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Error:      BOOL;
    ErrorID:    UDINT;
END_VAR

Name
Error
ErrorID

Type
BOOL
UDINT

Description
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

246

Version: 1.8.3

TF5810

MC_AxRtFinish_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

Behavior of the function block

The function block investigates the axis interface that has been passed to it every time it is called. A number
of problems can be detected and reported during this process:

• If one of the pointers has not been initialized the system responds with Error and

ErrorID:=dwTcHydErrCdPtrPlcMc or dwTcHydErrCdPtrMcPlc.

If these checks could be performed without problem, the control value for the axis is adapted according to
the values in Axis.ST_TcHydAxParam [} 130].

• The control value for the advance and the positional control reaction are combined to form the output

control value.

• Area compensation is taken into account.

• Compensation is applied for a bend in the characteristic curve.

• The overlap compensation, the terminal control value and the offset compensation are included in the

calculation.

If only the usual function blocks (encoder, generator, finish, drive) for the axis are to be called, a function
block of type MC_AxStandardBody_BkPlcMc [} 253] should be used for simplicity.

4.4.7.4

MC_AxRtFinishLinear_BkPlcMc

Available from version 3.0.16

The function block deals with the adjustment of the generated control value to the special features of the
axis, taking into account a characteristic curve.

 Inputs
VAR_INPUT
    EnableLinearisation:   BOOL;
    ValveTable:            POINTER TO LREAL:=0;
    ValveTableLowIdx:      INT:=0;
    ValveTableHighIdx:     INT:=0;
END_VAR

Name
EnableLinearisation
ValveTable

Type
BOOL
POINTER TO
LREAL

ValveTableLowIdx
ValveTableHighIdx

INT
INT

Description
TRUE at this input activates the linearization.
The address of the linearization table should be transferred
here. If possible, this should be the ValveCharacteristicTable of
an ST_TcMcAutoIdent [} 128] linked to the axis. If a NULL-
pointer is passed here the linearization table and the limiting
indices of the ST_TcMcAutoIdent structure associated with the
axis are used. If such a structure is not present, the function
block shows the behavior of a MC_AxRtFinish() function block.
The index of the first point in the linearization table.
The index of the last point in the linearization table. If possible,
this should be the ValveCharacteristicTblCount of an
ST_TcMcAutoIdent [} 128] linked to the axis.

TF5810

Version: 1.8.3

247

MC_AxRtFinishLinear_BkPlcMcEnableLinearisation  BOOLValveTable  Pointer To LREALValveTableLowIdx  INTValveTableHighIdx  INT↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

 Inputs/outputs

VAR_IN_OUT
    Axis:                  AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_
UTPUT
    Error:                 BOOL;
    ErrorID:               UDINT;
END_VAR

Name
Error
ErrorID

Type
BOOL
UDINT

Description
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

The function block investigates the axis interface that has been passed to it every time it is called. A number
of problems may be detected:

• EnableLinearisation is FALSE.

• There is no ValveTable available.

• ValveTableLowIdx is less than 0.

• ValveTableHighIdx is less than or equal to ValveTableLowIdx.

In these cases an MC_AxRtFinish_BkPlcMc [} 246] function block is called internally, and its outputs are
passed on. Otherwise the table linearization for the axis is performed. Note the following special
characteristics:

• The parameter for compensating the directional dependence (area ratio, gravity etc.) of the axis

velocity has no effect. This compensation should be taken into account in the table.

• The parameters for compensating a kink in the characteristic curve have no effect. This compensation

should be taken into account in the table.

• The parameter for the overlap compensation has no effect. This compensation should be taken into

account in the table.

• A pressing power output or an offset compensation cannot be realized through a linearization. The

corresponding parameters are active.

Example: Display of a linearization in the PlcMcManager:

248

Version: 1.8.3

TF5810

PLCopen Motion Control

A sample program can be found in the SampleList [} 374] of the Knowledge Base [} 320]. Demonstrates
automatic determination of a characteristic curve with an MC_AxUtiAutoIdent_BkPlcMc function block.

4.4.7.5

MC_AxRtGoErrorState_BkPlcMc

Available from version 3.0

(not recommended) This function block places the axis into a fault state.

 Inputs
VAR_INPUT
    Trigger:        BOOL;
    ErrorID:        UDINT;
    NoLogging:      BOOL;
END_VAR

Name
Trigger
ErrorID
NoLogging

Type
BOOL
UDINT
BOOL

Description
A rising edge at this input places the axis in a fault state.
An encoded indication of the cause of the error is provided here.
TRUE at this input suppresses the output of a message.

TF5810

Version: 1.8.3

249

MC_AxRtGoErrorState_BkPlcMcTrigger  BOOLErrorCode  DWORDNoLogging  BOOL↔Axis  Reference To AXIS_REF_BkPlcMcPLCopen Motion Control

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

Behavior of the function block

The axis is placed into a fault state by a rising edge at the Trigger input.

Requirements:

• The value at the ErrorCode input is not equal to 0.

• The axis is not already in an error state.

If NoLogging is FALSE (default state), message containing information on the affected axis and the
ErrorCode is generated during the transition of the axis to the error state. This default message
should be replaced with a message that is meaningful for the application. In this case the default
message should be suppressed by setting NoLogging to TRUE.

4.4.7.6

MC_AxRtMoveChecking_BkPlcMc

Available from version 3.0

The function block monitors the response of an axis.

 Inputs
VAR_INPUT
     Enable:       BOOL;
     MinDistance:  LREAL;
     Filter:       LREAL;
END_VAR

Name
Enable
MinDistance
Filter

Type
BOOL
LREAL
LREAL

Description
TRUE at this input activates the monitoring.
[mm] The required minimum distance must be transferred here.
[s] The measuring time must be specified here.

 Inputs/outputs

VAR_IN_OUT
     Axis:         AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

250

Version: 1.8.3

TF5810

MC_AxRtMoveChecking_BkPlcMcEnable  BOOLMinDistance  LREALFilter  LREAL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  TriggeredBOOL  TimeoutPLCopen Motion Control

 Outputs

VAR_OUPUT
     Triggered:    BOOL;
     Timeout:      BOOL;
END_VAR

Name
Triggered
Timeout

Type
BOOL
BOOL

Description
This output indicates that the axis was set to error state.
This output indicates that monitoring was triggered.

Behavior of the function block

The function block continuously checks whether the axis has traveled at least a MinDistance within Filter in
the direction that matches the required motion. If this is not the case, timeout is indicated. If Enable is
TRUE, the axis is set to error state dwTcHydErrCdNotMoving = 16#435D = 17245. This is indicated
through Triggered.

4.4.7.7

MC_AxRtSetDirectOutput_BkPlcMc

Available from version 3.0

The function block issues a control value, regardless of a profile generation.

 Inputs
VAR_INPUT
     Enable:           BOOL;
     OutValue:         LREAL;
     RampTime:         LREAL;
END_VAR

Name
Enable
OutValue
RampTime

Type
BOOL
LREAL
LREAL

Description
TRUE at this input activates the output.
The control value to be output should be transferred here.
[s] Here, the time should be specified in which the control value would reach full
scale.

 Inputs/outputs

VAR_IN_OUT
     Axis:             AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUPUT
     Busy:             BOOL;
     CommandAborted:   BOOL;

TF5810

Version: 1.8.3

251

MC_AxRtSetDirectOutput_BkPlcMcEnable  BOOLOutValue  LREALRampTime  LREAL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIdPLCopen Motion Control

     Error:            BOOL;
     ErrorID:          UDINT;
END_VAR

Name
Busy
CommandAborted
Error
ErrorID

Type
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
This indicates abortion of the function.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

A rising edge at Enable activates the function. The axis is put into states McState_Continousmotion [} 103]
and iTcHydStateExtGenerated [} 90] and Busy becomes TRUE. The control value of the axis is updated with
OutValue. The rate of change is specified through RampTime.

If Enable is set to FALSE, the control value is brought to 0.0 using RampTime, and the function is
terminated. Only then does Busy become FALSE.

If another function block takes over control of the axis while the MC_AxRtSetDirectOutput_BkPlcMc is
active, the function block terminates its function and indicates CommandAborted.

4.4.7.8

MC_AxRtSetExtGenValues_BkPlcMc

Available from version 3.0

The function block supplies an axis with command variables, which do not originate from the axis' own
generator.

 Inputs
VAR_INPUT
    Enable:             BOOL;
    Position:           LREAL:=0.0;
    Velocity:           LREAL:=0.0;
    TargetPosition:     LREAL:=0.0;
END_VAR

Name
Enable
Position
Velocity
TargetPosition

Type
BOOL
LREAL
LREAL
LREAL

Description
TRUE at this input activates the transfer of the command variables provided.
[mm] Set position value to be transferred cyclically.
[mm/s] Set velocity value to be transferred cyclically.
[mm] Target position value for the current motion to be transferred cyclically.

 Inputs/outputs

VAR_IN_OUT
    Axis:               AXIS_REF_BkPlcMc;
END_VAR

252

Version: 1.8.3

TF5810

MC_AxRtSetExtGenValues_BkPlcMcEnable  BOOLPosition  LREALVelocity  LREALTargetPosition  LREAL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ErrorUDINT  ErrorIdBOOL  ActiveName
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

PLCopen Motion Control

 Outputs

OUTPUT
    Error:              BOOL;
    ErrorID:            UDINT;
END_VAR

Name
Error
ErrorID

Type
BOOL
UDINT

Description
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

The function block investigates the axis interface that has been passed to it every time it is called. If a rising
edge is detected at Execute, the axis is put in state McState_Synchronizedmotion and
iTcHydStateExtGenerated.

If Execute is TRUE, the values of Position, Velocity and TargetPosition are entered in the runtime
variables of the axis. The purpose is to map the behavior of the generator function block for a comparable
motion, as far as possible.

If a falling edge is detected at Execute, the function block puts the axis in the state McState_Standstill. If
the axis is not at standstill at this time, it is stopped via the time-controlled ramp set in fStopRamp.

The generator function block of the axis should still be called cyclically. It deals with position control
and updates further internal variables.

4.4.7.9

MC_AxStandardBody_BkPlcMc

Available from version 3.0

This function block calls a function block of each of the following types: MC_AxRtEncoder_BkPlcMc [} 198],
MC_AxRuntime_BkPlcMc [} 237], MC_AxRtFinish_BkPlcMc [} 246] and MC_AxRtDrive_BkPlcMc [} 187].

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Error:      BOOL;
    ErrorID:    UDINT;
END_VAR

TF5810

Version: 1.8.3

253

MC_AxStandardBody_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

Name
Error
ErrorID

Type
BOOL
UDINT

Description
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

The usual components of the axis are called, depending on the value in ST_TcHydAxParam [} 130]. If one of
the called function blocks reports an Error, it will be returned with its ErrorID at the outputs of this function
block.

In the event of multiple problems, they are prioritized according to the following sequence: encoder,
generator, finish, drive.

4.4.7.10

MC_AxUtiStandardInit_BkPlcMc

Available from version 3.0

The function block handles the initialization and monitoring of axis components.

 Inputs
VAR_INPUT
    AxisName:           STRING(255);
    PathName:           STRING(255);
    pDeviceInput:       POINTER TO ST_TcPlcDeviceInput:=0;
    pDeviceOutput:      POINTER TO ST_TcPlcDeviceOutput:=0;
    pLogBuffer:         POINTER TO ST_TcPlcMcLogBuffer:=0;
    pStAxAuxLabels:     POINTER TO ST_TcMcAuxDataLabels:=0;
    pStAxAutoParams:    POINTER TO ST_TcMcAutoIdent;
    pStAxCommandBuf:    POINTER TO ST_TcPlcCmdBuffer_BkPlcMc:=0;    //from V3.0.8
    nLogLevel:          DINT:=0;
END_VAR

254

Version: 1.8.3

TF5810

MC_AxUtiStandardInit_BkPlcMcAxisName  STRING(255)PathName  STRING(255)pDeviceInput  Pointer To ST_TcPlcDeviceInputpDeviceOutput  Pointer To ST_TcPlcDeviceOutputpLogBuffer  Pointer To ST_TcPlcMcLogBufferpAuxLabels  Pointer To ST_TcMcAuxDataLabelspStAxAutoParams  Pointer To ST_TcMcAutoIdentpStAxLinCurveBuffer  Pointer To ST_TcMcAxLinCurveBufferpStAxCommandBuf  Pointer To ST_TcPlcCmdBuffer_BkPlcMcnLogLevel  DWORDfSaveDelay  LREAL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  ReadyBOOL  ErrorUDINT  ErrorIDBOOL  ReloadTablePLCopen Motion Control

Name
AxisName

Type
STRING

PathName

STRING

pDeviceInput

POINTER

pDeviceOutput

POINTER

pLogBuffer

POINTER

pStAxAuxLabels POINTER

pStAxAutoParam
s

pStAxCommandB
uf

POINTER

POINTER

nLogLevel

DINT

Description
Here, the text-based name of the axis (example: 'Axis_1') should be
transferred.
Here, the text-based path name (example: 'C:\TwinCAT\Project\'), under
which the axis parameters are to be saved, should be transferred.
This parameter is used to transfer the address of a variable of type
ST_TcPlcDeviceInput [} 149].
This parameter is used to transfer the address of a variable of type
ST_TcPlcDeviceOutput [} 153].

Here, the address of a variable of type ST_TcPlcMcLogBuffer [} 156] can
be transferred.

Here, the address of an ST_TcMcAuxDataLabels [} 149] structure with label
texts for customer-specific axis parameters can be transferred.

Here, the address of a variable of type ST_TcMcAutoIdent [} 128] can be
transferred.
From V3.0.8 the input BufferMode defined by the PLCopen is available
for various function blocks. The functionality that can be controlled with
this is currently in preparation. In this context this command buffer was
amended.

The input pStAxCommandBuf must currently not be supplied, or
only with the value 0.

Here, a coded value [} 347] should be transferred, which specifies the
threshold value for recording of messages.

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:       BOOL;
    Done:       BOOL;
    Ready:      BOOL;
    Error:      BOOL;
    ErrorID:    UDINT;
END_VAR

Name
Busy
Done
Ready

Type
BOOL
BOOL
BOOL

Error
ErrorID

BOOL
UDINT

Description
Indicates that a command is being processed.
This indicates that a command has been successfully processed.
This indicates that a command has been successfully processed and the axis
parameters have been successfully loaded.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

With each call, the function block examines the transferred axis interface and the transferred pointers. If a
change is detected, the function block indicates in the transferred AXIS_REF_BkPlcMc [} 86] structure that the
axis has to be reinitialized. The MC_AxParamLoad_BkPlcMc [} 287] function block used by this function block

TF5810

Version: 1.8.3

255

PLCopen Motion Control

will now automatically load the axis parameters from the file. If pAuxLabels is supplied, the label texts of the
customer-specific axis parameters are then loaded with a MC_AxParamAuxLabelsLoad_BkPlcMc [} 286]
function block.

The strings transferred as AxisName and PathName must not contain spaces or special characters,
which would make them unsuitable for generating a file name. The file name is generated by
concatenating the transferred strings and adding the extension '.dat'. The file name for the label
texts of the customer-specific axis parameter is generated in the same way, but with the extension
'.txt'.

The parameters pDeviceInput and pDeviceOutput should be supplied for all axes, which use an I/O
hardware for position detection. If virtual axes are used, these parameters should not be assigned
or assigned 0.

The input pStAxCommandBuf must currently not be supplied, or only with the value 0.

4.4.7.11

MC_AxRtCmdBufferExecute_BkPlcMc

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

Behavior of the function block

If the axis is equipped with a command buffer via an MC_AxUtiStandardInit() function block, positioning
commands such as MC_MoveAbsolute_BkPlcMc are entered in this buffer.

If iTcMc_ProfileCtrlBased is set as the setpoint generator, a function block of this type must be called
cyclically so that these commands are forwarded to the axis and actively processed.

4.4.8

Message logging

4.4.8.1

MC_AxRtLogAxisEntry_BkPlcMc

256

Version: 1.8.3

TF5810

MC_AxRtCmdBufferExecute_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcMC_AxRtLogAxisEntry_BkPlcMcpBuffer  Pointer To ST_TcPlcMcLogBufferLogLevel  DWORDSource  DWORDArgType  INTdiArg  DINTlrArg  LREALsArg  STRING(255)↔Axis  Reference To AXIS_REF_BkPlcMc↔Message  Reference To STRING(255)PLCopen Motion Control

Available from version 3.0

The function block enters an axis-related message in the LogBuffer of the library. Further information about
creating a log buffer can be found under FAQ #10 in the Knowledge Base [} 320].

All axis-related library function blocks use this function block for message outputs.

 Inputs
VAR_INPUT
    pBuffer:        POINTER TO ST_TcPlcMcLogBuffer;
    LogLevel:       DWORD:=0;
    Source:         DWORD:=0;
    Message:        STRING(255);
    ArgType:        INT:=0;
    diArg:          DINT:=0;
    lrArg:          LREAL:=0;
    sArg:           STRING(255);
END_VAR

Name
pBuffer

Type
POINTER

LogLevel

DWORD

Source

DWORD

Message
ArgType
diArg
lrArg
sArg

STRING
INT
DINT
LREAL
STRING

Description

Here the address of a variable of type ST_TcPlcMcLogBuffer [} 156] is to be
transferred.

A coded specification of the message type. A Logger Levels [} 347] value from
the Global Constants [} 338] should be used.

A coded specification of the message source. A Logger Sources [} 347] value
from the Global Constants [} 338] should be used.
The message text.
The type of the optional argument.
The value of the optional argument, if it is of type DINT.
The value of the optional argument, if it is of type LREAL.
The value of the optional argument, if it is of type STRING.

 Inputs/outputs

VAR_IN_OUT
    Axis:           POINTER TO AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
POINTER

Behavior of the function block

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc
[} 86] should be transferred

The only difference between the function block and MC_AxRtLogEntry_BkPlcMc [} 258] is that the axis name
appears before the message.

4.4.8.2

MC_AxRtLogClear_BkPlcMc

TF5810

Version: 1.8.3

257

MC_AxRtLogClear_BkPlcMcpBuffer  Pointer To ST_TcPlcMcLogBufferPLCopen Motion Control

Available from version 3.0

The function block deletes a LogBuffer of the library. Further information about creating a log buffer can be
found under FAQ #10 in the Knowledge Base [} 320].

 Inputs/outputs

VAR_IN_OUT
    pBuffer:        POINTER TO ST_TcPlcMcLogBuffer;
END_VAR

Name
pBuffer

Type
POINTER

Description
pBuffer: Here the address of a variable of type
ST_TcPlcMcLogBuffer [} 156] is to be transferred.

Behavior of the function block

All entries in the LogBuffer are deleted and initialized.

4.4.8.3

MC_AxRtLogEntry_BkPlcMc

Available from version 3.0

The function block enters a message in the LogBuffer of the library. Further information about creating a log
buffer can be found under FAQ #10 in the Knowledge Base [} 320].

 Inputs
VAR_INPUT
    pBuffer:        POINTER TO ST_TcPlcMcLogBuffer;
    LogLevel:       DWORD:=0;
    Source:         DWORD:=0;
    Message:        STRING(255);
    ArgType:        INT:=0;
    diArg:          DINT:=0;
    lrArg:          LREAL:=0;
    sArg:           STRING(255);
END_VAR

258

Version: 1.8.3

TF5810

MC_AxRtLogEntry_BkPlcMcpBuffer  Pointer To ST_TcPlcMcLogBufferLogLevel  DWORDSource  DWORDMessage  STRING(255)ArgType  INTdiArg  DINTlrArg  LREALsArg  STRING(255)PLCopen Motion Control

Name
pBuffer

Type
POINTER

LogLevel

DWORD

Source

DWORD

Message
ArgType
diArg
lrArg
sArg

STRING
INT
DINT
LREAL
STRING

Description

Here the address of a variable of type ST_TcPlcMcLogBuffer [} 156] is to be
transferred.

A coded specification of the message type. A Logger Levels [} 347] value from
the Global Constants [} 338] should be used.

A coded specification of the message source. A Logger Sources [} 347] value
from the Global Constants [} 338] should be used.
The message text.
The type of the optional argument.
The value of the optional argument, if it is of type DINT.
The value of the optional argument, if it is of type LREAL.
The value of the optional argument, if it is of type STRING.

Behavior of the function block

If pBuffer was supplied correctly and points to an ST_TcPlcMcLogBuffer [} 156], which has capacity for at
least one further message, the transferred message data are complemented with local time information and
entered in the message buffer.

4.4.8.4

MC_AxRtLoggerDeSpool_BkPlcMc

Available from version 3.0

The function block prevents overflowing of the LogBuffer of the library. Further information about creating a
log buffer can be found under FAQ #10 in the Knowledge Base [} 320].

 Inputs
VAR_INPUT
    Spare:      INT;
END_VAR

Name
spare

Type
INT

Description
The required number of free messages in the LogBuffer.

 Inputs/outputs

VAR_IN_OUT
    pBuffer:    POINTER TO ST_TcPlcMcLogBuffer;
END_VAR

Name
pBuffer

Type
POINTER

Behavior of the function block

Description
Here the address of a variable of type
ST_TcPlcMcLogBuffer [} 156] is to be transferred.

With each call the function block removes a message from the LogBuffer, if the number of free messages is
smaller than the minimum number transferred in Spare. An MC_AxRtLoggerSpool_BkPlcMc [} 261] function
block should be used to include the whole history in the Windows Event Viewer.

TF5810

Version: 1.8.3

259

MC_AxRtLoggerDespool_BkPlcMcpBuffer  Pointer To ST_TcPlcMcLogBufferSpare  INTPLCopen Motion Control

Using this function block makes sense whenever a write-restricted mass storage medium is used
(typically a FLASH DISK). As a minimum, a limited history of 10 to 15 messages is enabled.

4.4.8.5

MC_AxRtLoggerRead_BkPlcMc

Available from version 3.0

The function block reads a message from the LogBuffer of the library. Further information about creating a
log buffer can be found under FAQ #10 in the Knowledge Base [} 320].

This function block is used by diagnostics tools via ADS. A direct call from the PLC application
generally makes no sense.

 Inputs/outputs

VAR_IN_OUT
    Entry:      INT:=0;
    pBuffer:    POINTER TO ST_TcPlcMcLogBuffer;
    pEntry:     POINTER TO ST_TcPlcMcLogEntry;
END_VAR

Description
The number of the message to be read.

Here the address of a variable of type ST_TcPlcMcLogBuffer [} 156] is to be
transferred.

Here, the address of a variable of type ST_TcPlcMcLogEntry [} 156] should be
transferred as target.

Name
Entry
pBuffer

Type
INT
POINTER

pEntry

POINTER

 Outputs

VAR_OUTPUT
    Result:     DWORD:=0;
END_VAR

Name
Result

Type
DWORD

Description
Here, a coded cause of error is provided.

Behavior of the function block

The function block checks the transferred input values with each call. Two problems can be detected during
this process:

• If Entry is not in the valid range (1..20), the function block returns dwTcHydAdsErrInvalidIdxOffset as

Result.

• If pBuffer or pEntry are not defined, the function block returns dwTcHydAdsErrNotReady as Result.

If no problem was detected during the check, the function block copies the message selected by Entry from
the LogBuffer pBuffer into the message structure addressed with pEntry. Entry is understood as a relative
age specification: Entry:=1 selects the last message entered, Entry:=2 the next older one, etc. If the required
message is not available, an empty message is provided.

260

Version: 1.8.3

TF5810

MC_AxRtLoggerRead_BkPlcMcEntry  INTpBuffer  Pointer To ST_TcPlcMcLogBufferpEntry  Pointer To ST_TcPlcMcLogEntryDWORD  Result4.4.8.6

MC_AxRtLoggerSpool_BkPlcMc

PLCopen Motion Control

Available from version 3.0

The function block deals with transferring messages from the LogBuffer of the library into the Windows Event
Viewer. Further information about creating a log buffer can be found under FAQ #10 in the Knowledge Base
[} 320].

 Inputs/outputs

VAR_IN_OUT
    pBuffer:        POINTER TO ST_TcPlcMcLogBuffer;
END_VAR

Name
pBuffer

Type
POINTER

Description

Here the address of a variable of type ST_TcPlcMcLogBuffer [} 156] is to be
transferred.Description

Behavior of the function block

With each call the function block removes a message from the LogBuffer and transfers it to the Windows
Event Viewer.

If the computer uses a write-restricted mass storage medium (typically FLASH DISK), it makes no sense to
use the Windows Event Viewer. An MC_AxRtLoggerDespool_BkPlcMc [} 259] function block can be used to
generate a history in any case.

4.4.9

Utilities

4.4.9.1

MC_AxParamDelayedSave_BkPlcMc

The function block triggers a time-delayed writing of the parameter file after a parameter has been changed.

 Inputs
VAR_INPUT
    Delay:     LREAL:=0.0;
END_VAR

Name
Delay

Type
LREAL

Description
[s] Delay when triggering the parameter backup.

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

TF5810

Version: 1.8.3

261

MC_AxRtLoggerSpool_BkPlcMcpBuffer  Pointer To ST_TcPlcMcLogBufferMC_AxParamDelayedSave_BkPlcMcDelay  LREAL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ErrorUDINT  ErrorIdPLCopen Motion Control

Name
Axis

Type
AXIS_REF_BkPlcMc

 Outputs

VAR_OUTPUT
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

Name
Error
ErrorID

Type
BOOL
UDINT

Description
This output indicates any problems that may have occurred.
In the event of an error, coded information about the type of problem is reported here.

Behavior of the function block

If one of the function blocks listed below noticeably changes a parameter, a time is reset. This time is
increased by the cycle time in every cycle where this is not the case. If it reaches the parameter Delay, the
writing of the parameter file is triggered.

• MC_WriteBoolParameter_BkPlcMc

• MC_WriteParameter_BkPlcMc

• MC_AxUtiAutoIdent_BkPlcMc

• MC_LinTableImportFromAsciFile_BkPlcMc

• MC_LinTableImportFromBinFile_BkPlcMc

4.4.9.2

MC_AxRtCommandsLocked_BkPlcMc : DWORD

The function simplifies setting and deleting of a protective function in the status double word of an axis.

 Inputs
VAR_INPUT
     nStateDWord:  DWORD:=0;
     bProtect:     BOOL:=FALSE;
END_VAR

Name
nStateDWord
bProtect

Type
DWORD
BOOL

Description
The current state of the status double word.
The required state of the protective function.

Behavior of the function

The function shows the status bit of the protective function in the transferred status double word, depending
on bProtect.

The application must assign the result of the function to the status double word of the axis.

There is an example [} 378] available.

262

Version: 1.8.3

TF5810

4.4.9.3

MC_AxRtFollowUp_BkPlcMc

PLCopen Motion Control

The function block updates the offset compensation.

 Inputs
VAR_INPUT
     Enable:       BOOL;
END_VAR

Name
Enable

Type
BOOL

Description
A TRUE at this input enables the function block.

 Inputs/outputs

VAR_IN_OUT
     Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Behavior of the function block

Description

Here, the address of a variable of type Axis_Ref_BkPlcMc
[} 86] should be transferred.

If Enable = TRUE, the actual position is copied to all target and set positions. Target velocity, lag error and
position controller output are set to 0.0. With a falling edge at Enable, the axis will re-enable the position
controller depending on parameters and enables at the current actual position.

The function block should not be enabled for an axis that is performing an active movement or needs to be
controlled.

Because the position control is disabled, the axis can drift.

If the axis is to be moved by external actions, the required oil paths must be opened by the application.

4.4.9.4

MC_AxRtUpdateExternalEncoder_BkPlcMc

The function block updates the actual position of an axis with a value determined by the application.

 Inputs
VAR_INPUT
     ActualPos:  LREAL;
END_VAR

Name
ActualPos

Type
LREAL

Description
The new value for the actual position.

 Inputs/outputs

VAR_IN_OUT
     Axis:       AXIS_REF_BkPlcMc;
END_VAR

TF5810

Version: 1.8.3

263

MC_AxRtFollowUp_BkPlcMcEnable  BOOL↔Axis  Reference To AXIS_REF_BkPlcMcMC_AxRtUpdateExternalEncoder_BkPlcMcActualPos  LREAL↔Axis  Reference To AXIS_REF_BkPlcMcPLCopen Motion Control

Name
Axis

Type
AXIS_REF_BkPlcMc

Behavior of the function block

Description

Here, the address of a variable of type Axis_Ref_BkPlcMc [} 86]
should be transferred.

ActualPos is entered as an actual position in the runtime data of the axis. The change generated by this in
relation to the cycle time of the axis is used as the actual velocity. bEncoderResponse is set in stAxRtData to
mark an update that has been made.

NOTICE
If this function block is used, no MC_AxRtEncoder_BkPlcMc() or MC_AxStandardBody_BkPlcMc() function
block may be called.

The actual position must be updated in each cycle.

NOTICE

The function block does not filter.

4.4.9.5

MC_AxUtiOffsetLatch_BkPlcMc

Available from version 3.0.40

The function block updates the offset compensation.

 Inputs
VAR_INPUT
     Execute:      BOOL;
     OffsetLimit:  LREAL:=0.5;
END_VAR

Name
Execute
OffsetLimit

Type
BOOL
LREAL

Description
A rising edge triggers the update.
[V] The maximum permissible value range for the offset compensation.

 Inputs/outputs

VAR_IN_OUT
     Axis:         AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type Axis_Ref_BkPlcMc [} 86] should
be transferred.

264

Version: 1.8.3

TF5810

MC_AxUtiOffsetLatch_BkPlcMcExecute  BOOLOffsetLimit  LREAL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  DoneBOOL  ErrorUDINT  ErrorIDBOOL  LatchedLREAL  OffsetPLCopen Motion Control

 Outputs

VAR_OUTPUT
     Done:         BOOL;
     Error:        BOOL;
     ErrorId:      UDINT;
     Latched:      BOOL;
     Offset:       LREAL;
END_VAR

Name
Done
Error
ErrorId
Latched
Offset

Type
BOOL
BOOL
UDINT
BOOL
LREAL

Description
A successful update is indicated here.
This output indicates any problems that may have occurred.
In the event of an error, coded information about the type of problem is reported here.
This output signals that the update was successfully completed.
[V] This output reports the offset value. It is only accepted as a new compensation
when Done.

Behavior of the function block

With a rising edge at Execute, Offset is updated with the current output of the position controller.

Before accepting this value as compensation, the function block checks for several possible errors:

• The axis must have a controller enable and must not be in an active motion state or error state

(Axis.stAxRtData.iCurrentStep=iTcHydStateIdle). (error code dwTcHydAdsErrBusy)

• The detected controller output must not be outside ±OffsetLimit. (error code

dwTcHydAdsErrIllegalValue)

If one of the errors has occurred, Error is reported and ErrorId is assigned the specified error code. In this
case, the compensation remains unchanged.

Otherwise, offset is applied as the new compensation value. Since from this point in time the part of the
output value previously provided by the position controller is taken over by the compensation, the output of
the controller must be canceled. The following steps are carried out once only:

• The set and current target positions are updated with the actual position.

• The position error (lag error) is set to zero.

• The position controller output is set to zero.

• The I part of the default position controller is wiped.

• If a position controller other than the default position controller is used, its I part must be deleted by the

application.

All outputs are set to the idle state with a falling edge at Execute.

4.4.9.6

Filters

4.4.9.6.1

MC_AxUtiAverageDerivative_BkPlcMc

Available from version 3.0

TF5810

Version: 1.8.3

265

MC_AxUtiAverageDerivative_BkPlcMcInput  LREALMinIdx  DINTMaxIdx  DINTpBuffer  Pointer To LREAL↔Axis  Reference To AXIS_REF_BkPlcMcLREAL  OutputBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

The function block determines the derivative of a value through numeric differentiation over more than one
cycle.

 Inputs
VAR_INPUT
    Input:      LREAL:=0.0;
    MinIdx:     DINT:=0;
    MaxIdx:     DINT:=0;
    pBuffer:    POINTER TO LREAL:=0;
END_VAR

Name
Input
MinIdx
MaxIdx
pBuffer

Type
LREAL
DINT
DINT
POINTER

Description
The raw value of the parameter to be filtered.
The index of the first filter buffer element to be used.
The index of the last filter buffer element to be used.
The address of the first filter buffer element.

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Output:     LREAL:=0.0;
    Error:      BOOL:=FALSE;
    ErrorID:    UDINT:=0;
END_VAR

Name
Output
Error
ErrorID

Type
LREAL
BOOL
UDINT

Description
The filtered value.
This output indicates problems with the transferred parameters.
In the event of an error, coded information about the type of problem is reported here.

Behavior of the function block

With each call the function block checks the address of the filter buffer pBuffer and the indices of the
elements MinIdx and MaxIdx to be used. If the transferred values are obviously nonsensical, the system
responds with Error and coded information in ErrorID. Otherwise, with each call Input is entered in the filter
buffer, and the average value of the modification over the set of values available in the buffer is formed and
returned as Output.

The set of values for averaging contains (MaxIdx - MinIdx + 1) values. The measuring time is
determined by multiplication of this number with the cycle time.

The principle of sliding averaging leads to a delay amounting to half the measuring time. If the
filtered parameter can be used in a control loop, the resulting frequency-dependent phase shift can
lead to restrictions for the parameter selection.

The function block has no way to fully check the transferred values of pBuffer, MinIdx and MaxIdx.
The user must ensure that these values can be used safely. Otherwise may behave in an
unpredictable manner (overwriting of memory) or abortion of the PLC operation.

266

Version: 1.8.3

TF5810

4.4.9.6.2

MC_AxUtiPT1_BkPlcMc

PLCopen Motion Control

Available from version 3.0

The function block calculates a first-order low-pass.

 Inputs
VAR_INPUT
     fInput:       LREAL:=0.0;
     fCycletime:   LREAL:=0.001;
     fT0:          LREAL:=1.0;
END_VAR

Name
fInput
fCycletime
fT0

Type
LREAL
LREAL
LREAL

Description
The raw value of the parameter to be filtered.
[s] The cycle time of the calling PLC task.
[s] The filter time constant.

 Outputs

VAR_OUTPUT
     fOutput:      LREAL;
     bError:       BOOL;
     nErrorId:     UDINT;
END_VAR

Name
fOutput
bError
nErrorId

Type
LREAL
BOOL
UDINT

Description
The filtered value.
This output indicates problems with the transferred parameters.
In the event of an error, coded information about the type of problem is reported here.

Behavior of the function block

With each call the function block checks the transferred parameters. If an invalid value is detected, the
function block responds with bError and a corresponding value in nErrorId. Otherwise, the internal variables
are updated with fInput, and the filtered value is returned as fOutput.

4.4.9.6.3

MC_AxUtiPT2_BkPlcMc

Available from version 3.0

The function block calculates a second-order low-pass.

TF5810

Version: 1.8.3

267

MC_AxUtiPt1_BkPlcMcfInput  LREALfCycletime  LREALfT0  LREALLREAL  fOutputBOOL  bErrorUDINT  nErrorIdMC_AxUtiPt2_BkPlcMcfInput  LREALfCycletime  LREALfTheta  LREALfT0  LREALLREAL  fOutputBOOL  bErrorUDINT  nErrorIdPLCopen Motion Control

 Inputs
VAR_INPUT
     fInput:       LREAL:=0.0;
     fCycletime:   LREAL:=0.001;
     fTheta:       LREAL:=1.0;
     fT0:          LREAL:=1.0;
END_VAR

Name
fInput
fCycletime
fTheta
fT0

Type
LREAL
LREAL
LREAL
LREAL

Description
The raw value of the parameter to be filtered.
[s] The cycle time of the calling PLC task.
The attenuation.
The filter time constant.

 Outputs

VAR_OUTPUT
     fOutput:      LREAL;
     bError:       BOOL;
     nErrorId:     UDINT;
END_VAR

Name
fOutput
bError
nErrorId

Type
LREAL
BOOL
UDINT

Description
The filtered value.
This output indicates problems with the transferred parameters.
In the event of an error, coded information about the type of problem is reported here.

Behavior of the function block

With each call the function block checks the transferred parameters. If an invalid value is detected, the
function block responds with bError and a corresponding value in nErrorId. Otherwise, the internal variables
are updated with fInput, and the filtered value is returned as fOutput.

4.4.9.6.4

MC_AxUtiSlewRateLimitter_BkPlcMc

Available from version 3.0

The function block generates a rise-limited ramp.

 Inputs
VAR_INPUT
    fInput:       LREAL:=0.0;
    fCycletime:   DINT:=0;
    fMaxRate:     DINT:=0;
END_VAR

Name
fInput
fCycletime
fMaxRate

Type
LREAL
DINT
DINT

Description
The raw value of the parameter to be filtered.
[s] The cycle time of the calling PLC task in seconds.
The magnitude of the maximum permitted rate of change at the output as change
per second.

268

Version: 1.8.3

TF5810

MC_AxUtiSlewRateLimitter_BkPlcMcfInput  LREALfCycletime  LREALfMaxRate  LREALLREAL  fOutputBOOL  bErrorUDINT  nErrorIdPLCopen Motion Control

 Outputs

VAR_OUTPUT
    fOutput:      LREAL:=0.0;
    bError:       BOOL:=FALSE;
    nErrorId:     UDINT:=0;
END_VAR

Name
fOutput
bError
nErrorId

Type
LREAL
BOOL
UDINT

Description
[1/s] The filtered value.
This output indicates problems with the transferred parameters.
In the event of an error, coded error information is output here.

Behavior of the function block

With each call the function block checks the transferred values for fCycletime and fMaxRate. If the values
are incorrect, the system responds with bError and coded information in nErrorId. Otherwise, the difference
between Input and Output is determined with each call. If the magnitude of this difference is less than or
equal to fMaxRate * fCycletime, the value of Input is used directly as fOutput. Otherwise, fOutput is
changed by fMaxRate * fCycletime. The sign is selected automatically.

The value for fCycletime must be ≥ 0.001. Negative values are not permitted for fMaxRate.

Input will usually be a value, which is determined and filtered based on the cycle of the axis blocks.
AXIS_REF_BkPlcMc [} 86].ST_TcHydAxParam [} 130].fCycletime can be used for fCycletime here.

4.4.9.6.5

MC_AxUtiSlidingAverage_BkPlcMc

Available from version 3.0

The function block determines a sliding average value.

 Inputs
VAR_INPUT
    Input:      LREAL:=0.0;
    MinIdx:     DINT:=0;
    MaxIdx:     DINT:=0;
    pBuffer:    POINTER TO LREAL:=0;
END_VAR

Name
Input
MinIdx
MaxIdx
pBuffer

Type
LREAL
DINT
DINT
POINTER

Description
The raw value of the parameter to be filtered.
The index of the first filter buffer element to be used.
The index of the last filter buffer element to be used.
The address of the first filter buffer element.

TF5810

Version: 1.8.3

269

MC_AxUtiSlidingAverage_BkPlcMcInput  LREALMinIdx  DINTMaxIdx  DINTpBuffer  Pointer To LREALLREAL  OutputLREAL  DerivativePLCopen Motion Control

 Outputs

VAR_OUTPUT
    Output:     LREAL:=0.0;
END_VAR

Name
Output

Type
LREAL

Description
Output: The filtered value.

Behavior of the function block

With each call the function block checks the address of the filter buffer pBuffer and the indices of the
elements MinIdx and MaxIdx to be used. If the transferred values are obviously nonsensical, Input is output
as Output. Otherwise, with each call Input is entered in the filter buffer, and the average value over the set
of values available in the buffer is formed and returned as Output.

The set of values for averaging contains (MaxIdx - MinIdx + 1) values. The filter time is determined
by multiplication of this number with the cycle time.

The principle of sliding averaging leads to a delay amounting to half the filter time. If the filtered
parameter can be used in a control loop, the resulting frequency-dependent phase shift can lead to
restrictions for the parameter selection.

The function block has no way to fully check the transferred values of pBuffer, MinIdx and MaxIdx.
The user must ensure that these values can be used safely. Otherwise may behave in an
unpredictable manner (overwriting of memory) or abortion of the PLC operation.

4.4.9.7

Identification

4.4.9.7.1

MC_AxUtiAutoIdent_BkPlcMc

Available from version 3.3.6.4

The function block automatic determines a number of axis parameters.

 Inputs
VAR_INPUT
    Execute: BOOL;
    Wait: BOOL;
END_VAR

270

Version: 1.8.3

TF5810

MC_AxUtiAutoIdent_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcExecute  BOOLWait  BOOLBOOL  BusyBOOL  DoneBOOL  ErrorUDINT  ErrorIDINT  StepBOOL  InRecoveryBOOL  RequestPositivBOOL  RequestNegativName
Execute
Wait

Type
BOOL
BOOL

Description
A rising edge at this input triggers the identification.
(From TwinCAT 2 V3.0.44 / TwinCAT 3 V3.3.1.22) If this input is set to TRUE, the
internal sequence processor does not go outside the recovery time. This prevents the
output value from ramping up when the valves are not yet switched.

PLCopen Motion Control

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Inputs/outputs

VAR_IN_OUT
    Axis: AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

 Outputs

VAR_OUTPUT
    Busy: BOOL;
    Done: BOOL;
    Error: BOOL;
    ErrorID: UDINT;
    Step: INT;
    InRecovery: BOOL;
END_VAR

Name
Busy
Done
Error
ErrorID
Step
InRecovery

RequestPositiv
e
RequestNegati
ve

Type
BOOL
BOOL
BOOL
UDINT
INT
BOOL

BOOL

Description
Indicates that a command is being processed.
This indicates successful identification.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.
The current step of the internal sequence processor.
(From TwinCAT 2 V3.0.44 / TwinCAT 3 V3.3.1.22) This indicates that a recovery
time (defined by ValveCharacteristicRecovery) has elapsed.
This signals a movement in a positive direction. (from version 3.3.6.4)

BOOL

This signals a movement in a negative direction. (from version 3.3.6.4)

Behavior of the function block

The function block checks whether the pointer IS_REF_BkPlcMc [} 86].pStAxAutoParams was initialized. If this
is the case, it performs a number of initializations when a rising edge is detected at the Execute input and
then commences the parameter identification. The individual steps of the identification are determined by the
values in ST_TcMcAutoIdent [} 128].

EnableEndOfTravel: If this boolean parameter is set, the mechanical travel limits are determined
automatically. First, the system ensures that the axis can move freely or is at the positive function block. The
axis is now moved with a negative control voltage until it has reached the function block. The axis is then
operated with a positive control voltage until the positive function block has been detected. The control
voltage is limited to EndOfTravel_NegativLimit and EndOfTravel_PositivLimit. If the positive travel limit
falls below the negative limit, the values are swapped, and Axis.stAxParams.bDrive_Reversed is inverted.

EnableOverlap, EnableZeroAdjust: If one of these boolean parameter is set, the cover or the offset voltage
of the valve is determined.

Thus operation is influenced by EndOfTravel_Negative and EndOfTravel_Positive.

TF5810

Version: 1.8.3

271

PLCopen Motion Control

First, the axis is moved into a position window that is located in the middle between EndOfTravel_Positiv
and EndOfTravel_Negativ. The width of the window is 80% of the area defined by these parameters. The
output polarity of the drive is inverted, if required. Now, the output voltage is determined, at which the axis
starts moving in positive direction. Then, the corresponding negative voltage is determined. By offsetting
these parameter, both the cover and the offset voltage are determined. The type of entry in the axis
parameter is controlled through EnableOverlap and EnableZeroAdjust.

EnableArreaRatio: If this boolean parameter is set, the direction-dependent velocity ratio is determined.
First, the axis is moved into a position window, which is located in the middle between pStAxAutoParams.
EndOfTravel_Positiv and pStAxAutoParams. EndOfTravel_Negativ. The width of the window is 80% of the
area defined by these parameters. Then, the axis is moved in positive and negative direction for one second
with a control voltage of 1 V. The velocities determined during this movement are divided, in order to
determine the velocity ratio.

EndOfTravel_Negativ: [mm] If determination of the travel limits is activated, this value is determined by the
function block. If it is disabled, the application must specify the value here.

This parameter influences the determination of the offset voltage and the area ratio.

EndOfTravel_Positiv: [mm] If determination of the travel limits is activated, this value is determined by the
function block. If it is disabled, the application must specify the value here.

This parameter influences the determination of the offset voltage and the area ratio.

EndOfIncrements_Negativ: [1] If determination of the travel limits is activated, this value is determined by
the function block. It then matches EndOfTravel_Negativ, but it is the raw encoder value in increments.

EndOfIncrements_Positiv: [1] If determination of the travel limits is activated, this value is determined by
the function block. It then matches EndOfTravel_Positiv, but it is the raw encoder value in increments.

EndOfTravel_NegativLimit: [V] This parameter limits negative output voltages.

EndOfTravel_PositivLimit: [V] This parameter limits positive output voltages.

EndOfTravel_PositivDone: This signal is set by the function block, if determination of the travel limits is
disabled or the positive travel limit was determined.

EndOfTravel_NegativDone: This signal is set by the function block, if determination of the travel limits is
disabled or the negative travel limit was determined.

EndOfVelocity_NegativLimit: [mm/s] This parameter limits negative velocities. If this velocity is reached or
exceeded during the measurement, the current measurement is completed, but no further measurement in
this direction is performed.

EndOfVelocity_PositivLimit: [mm/s] This parameter limits positive velocities. If this velocity is reached or
exceeded during the measurement, the current measurement is completed, but no further measurement in
this direction is performed.

DecelerationFactor: [1] After the measuring stroke, the axis is moved to the end of the measuring path for
the next measuring stroke. The regular axis parameter fMaxAcc and fMaxDec, which are weighted with this
factor, are used.

EnableValveCharacteristic: If this boolean parameter is set, the characteristic velocity curve is determined
automatically.

ValveCharacteristicTable: This ARRAY[1..2,1..100] contains the value pairs of the linearization table.
ValveCharacteristicTable[nnn,1] is the normalized velocity value, ValveCharacteristicTable[nnn,2] is the
normalized output value. Within the table, the value pairs with increasing index have increasing values for
the velocity value and the output value. The first value pair therefore describes the fastest negative point, the
last active value pair the fastest positive point. During automatic determination, the control voltage is limited

272

Version: 1.8.3

TF5810

PLCopen Motion Control

to EndOfTravel_NegativLimit and EndOfTravel_PositivLimit and the velocity to
EndOfVelocity_NegativLimit and EndOfVelocity_PositivLimit. The further points of the table are
determined through extrapolation from the last two measuring points.

ValveCharacteristicType: The identification can be adapted here to special valve variants or special
conditions of the axis. See also E_TcMcValveType [} 120].

ValveCharacteristicTblCount: This parameter specifies the number of value pairs to be determined in
ValveCharacteristicTable. The value must be odd and between 3 and 99 (including).

ValveCharacteristicLowEnd: [mm] The lower end position of the range permitted for determining the
characteristic curve.

ValveCharacteristicHighEnd: [mm] The upper end position of the range permitted for determining the
characteristic curve.

ValveCharacteristicRamp: [s] This parameter specifies the ramp for establishing the measuring voltage for
the characteristic curve determination. During the specified time the voltage is increased to 10 V. Since the
actual voltages are generally lower, less time is usually required to establish the voltage. Please pay
attention to the notes at the end of this document.

ValveCharacteristicSettling: [s] Once the control value has been ramped up to the test level for the
measurement, the starting of the measurement can be delayed through this parameter. Please pay attention
to the notes at the end of this document.

ValveCharacteristicRecovery: [s] This parameter defines a dwell time, which is adhered to before the
measurement travel. This enables the supply to compensate a pressure drop that may have been caused by
the previous measurement travel.

During this time the axis is not controlled.

From TwinCAT 2 V3.0.44 / TwinCAT 3 V3.3.1.22: The expiry of the dwell time is indicated at the InRecovery
output.

ValveCharacteristicMinCycle: [mm] The measurement travel is only valid if the measuring voltage was
established before the axis has moved towards the center of the measuring distance defined by
ValveCharacteristicHighEnd and ValveCharacteristicLowEnd by less than the half of this value.
Otherwise, the effective measuring distance (without ramps) is less than this distance, and this measurement
and all further measurements in this direction are replaced by a value calculated using the reference velocity
of the axis.

Valve_LinLimitP, Valve_LinLimitM: [mm/s] The lowest velocity for using the linearization table. For lower
velocities, the characteristic curve is replaced by a straight line that connects the zero point with the point for
the velocity specified here.

Example: Diagram of a characteristic curve determination in TwinCAT ScopeView:

TF5810

Version: 1.8.3

273

PLCopen Motion Control

Example: Display of a linearization in the PlcMcManager:

274

Version: 1.8.3

TF5810

PLCopen Motion Control

The characteristic curve determined in this way can be used with an MC_AxRtFinishLinear_BkPlcMc
function block for linearization at runtime.

The characteristic curve is stored in the parameter file of the axis and automatically read on system
startup.

Irrespective of that, the linearization table can be imported from a text or binary file [} 336] with an
MC_LinTableImportFromAsciFile_BkPlcMc [} 277] or MC_LinTableImportFromBinFile_BkPlcMc [} 278] function
block, or exported with an MC_LinTableExportToAsciFile_BkPlcMc [} 275] or
MC_LinTableExportToBinFile_BkPlcMc [} 276] function block.

If a lower velocity than at the previous measuring point is detected in the same direction during the
measurement at a test output, a warning is issued regardless of the set logger limit. The measuring
point is automatically corrected to avoid falling characteristic ranges. This correction has no
influence on the validity of the characteristic curve. However, it should be checked whether the
values in ValveCharacteristicRamp and ValveCharacteristicSettling are suitable for this axis.

4.4.9.7.2

MC_LinTableExportToAsciFile_BkPlcMc

The function block exports a linearization table to a file in ASCI format.

TF5810

Version: 1.8.3

275

MC_LinTableExportToAsciFile_BkPlcMcExecute  BOOLFileName  STRING(255)↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

 Inputs
VAR_INPUT
    Execute:     BOOL:=FALSE;
    FileName:    STRING(255):='';
END_VAR

Name
Execute
FileName

Type
BOOL
STRING

Description
A rising edge initiates the export.

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    Done:           BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc
[} 86] should be transferred.

Name
Busy
Done
Error
ErrorID

Type
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
This indicates successful identification.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

The function block generates a file in the format expected by MC_LinTableImportFromAsciFile_BkPlcMc
[} 277].

4.4.9.7.3

MC_LinTableExportToBinFile_BkPlcMc

The function block exports a linearization table to a file in binary format.

 Inputs
VAR_INPUT
    Execute:     BOOL:=FALSE;
    FileName:    STRING(255):='';
END_VAR

Name
Execute
FileName

Type
BOOL
STRING

Description
A rising edge initiates the export.

276

Version: 1.8.3

TF5810

MC_LinTableExportToBinFile_BkPlcMcExecute  BOOLFileName  STRING(255)↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    Done:           BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
Busy
Done
Error
ErrorID

Type
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
This indicates successful identification.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

The function block generates a file in the format expected by MC_LinTableImportFromBinFile_BkPlcMc
[} 278].

4.4.9.7.4

MC_LinTableImportFromAsciFile_BkPlcMc

The function block imports a linearization table from a file in ASCI format.

 Inputs
VAR_INPUT
    Execute:     BOOL:=FALSE;
    FileName:    STRING(255):='';
END_VAR

Name
Execute
FileName

Type
BOOL
STRING

Description
A rising edge initiates the import.

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

TF5810

Version: 1.8.3

277

MC_LinTableImportFromAsciFile_BkPlcMcExecute  BOOLFileName  STRING(255)↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  ErrorUDINT  ErrorIDINT  LastIdxPLCopen Motion Control

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    Done:           BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
    LastIdx:        INT:=0;
END_VAR

Name
Busy
Done
Error
ErrorID
LastIdx

Type
BOOL
BOOL
BOOL
UDINT
INT

Description
Indicates that a command is being processed.
This indicates successful identification.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

The function block reads the contents of the file and interprets it line by line according to the following rules:

• Leading spaces (blanks, tabs) are ignored.

• The digits 0..9 are accepted and used as digits before the decimal point for the first value.

• A point or comma is interpreted as a separator between digits before and after the decimal point.

• The digits 0..9 are accepted and used as digits after the decimal point for the first value.

• Spaces (blanks, tabs) are interpreted as separators between the first and second values.

• The digits 0..9 are accepted and used as digits before the decimal point for the second value.

• A point or comma is interpreted as a separator between digits before and after the decimal point.

• The digits 0..9 are accepted and used as digits after the decimal point for the second value.

• If unexpected characters are detected or expected elements are missing, the import is aborted with an

error.

• Each pair of numbers is entered as a point in the linearization table of the axis. LastIdx is thereby

incremented. After a successful import, the number of table points read is available here.

Manipulation of the file

The ASCI format makes it easy to manipulate such a file with a simple editor. This is possible, but not
recommended. A deviation from the expected structure of the file makes it impossible to import it. Even with
the correct formatting, however, a linearization table can inadvertently be created that makes it impossible
for the system to function correctly. In addition, persons may be endangered and the products or plant may
be damaged.

4.4.9.7.5

MC_LinTableImportFromBinFile_BkPlcMc

The function block imports a linearization table from a file in binary format.

 Inputs
VAR_INPUT
    Execute:     BOOL:=FALSE;
    FileName:    STRING(255):='';
END_VAR

278

Version: 1.8.3

TF5810

MC_LinTableExportToBinFile_BkPlcMcExecute  BOOLFileName  STRING(255)↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

Name
Execute
FileName

Type
BOOL
STRING

Description
A rising edge initiates the import.

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    Done:           BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
    LastIdx:        INT:=0;
END_VAR

Name
Busy
Done
Error
ErrorID
LastIdx

Type
BOOL
BOOL
BOOL
UDINT
INT

Description
Indicates that a command is being processed.
This indicates successful identification.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

The function block reads the contents of the file and interprets it line by line according to the following rules:

• Each two values are regarded as a pair of numbers.

• Each pair of numbers is entered as a point in the linearization table of the axis. LastIdx is thereby

incremented. After a successful import, the number of table points read is available here.

Manipulation of the file

The binary format makes it practically impossible to manipulate such a file with a simple editor. This is
possible with an appropriate program, but absolutely not recommended. The expected structure of the file is
hardly visible and it is practically impossible to maintain the formatting. Even with the correct formatting,
however, it will hardly be possible to create a usable linearization table. The correct function of the system
will not be possible. In addition, persons may be endangered and the products or plant may be damaged.

4.5

Parameter

4.5.1

MC_AxAdsCommServer_BkPlcMc

TF5810

Version: 1.8.3

279

MC_AxAdsCommServer_BkPlcMcnFirstAxisIndex  INTnLastAxisIndex  INTpAxItf  Pointer To AXIS_REF_BkPlcMcBOOL  PlcMcManOfflinePLCopen Motion Control

Available from version 3.0

The function block gives the application the capacity to function as an ADS server. Calls function blocks of
type MC_AxAdsReadDecoder_BkPlcMc [} 283] and MC_AxAdsWriteDecoder_BkPlcMc [} 285] as required.
The ADS codes [} 345] are listed in the Knowledge Base.

 Inputs
VAR_INPUT
    nFirstAxisIndex:    INT;
    nLastAxisIndex:     INT;
END_VAR

Name
nFirstAxisIndex
nLastAxisIndex

Type Description
INT
INT

This parameter is used to specify the dimensioning of the AXIS_REF_BkPlcMc
[} 86] array.

An incorrect specification at this point excludes some of the axes from
the communication or results in a crash of the PLC application by
triggering serious runtime errors (Page Fault Exception).

 Inputs/outputs

VAR_IN_OUT
    pAxItf:             POINTER TO AXIS_REF_BkPlcMc;
END_VAR

Name
pAxItf

Type
POINTER

Description

Here, the address of a variable or an array of variables of type AXIS_REF_BkPlcMc
[} 86] should be transferred.

 Outputs

VAR_OUTPUT
    PlcMcManOffline:    BOOL;
END_VAR

Name
PlcMcManOffline

Type
BOOL

Behavior of the function block

Description
This parameter corresponds to the parameter of the same name of the
MC_AxAdsCommServer_BkPlcMc [} 279] function block.

Through cyclic calling of this function block in the PLC application, the application assumes the character of
an ADS server and responds to ADS read and ADS write-access like any other ADS server. This includes
the decoding of IdxGroup/IdxOffset addressing. Function blocks of type MC_AxAdsReadDecoder_BkPlcMc
[} 283] and MC_AxAdsWriteDecoder_BkPlcMc [} 285] are called as required.

This function block must not be used if the PLC application already is an ADS server.

In this case the function blocks of type MC_AxAdsReadDecoder_BkPlcMc [} 283] and
MC_AxAdsWriteDecoder_BkPlcMc [} 285] should be called from the existing ADS server function block of the
application.

280

Version: 1.8.3

TF5810

4.5.2

MC_Communications_BkPlcMc

PLCopen Motion Control

The function block internally calls the function blocks MC_AxAdsCommServer_BkPlcMc [} 279] and
MC_AxRtLoggerSpool_BkPlcMc [} 261]. It also provides a message buffer.

 Inputs
VAR_INPUT
    nFirstAxisIndex:    INT;
    nLastAxisIndex:     INT;
    pAxItf:             POINTER TO AXIS_REF_BkPlcMc;
END_VAR

Name
nFirstAxisIndex
nLastAxisIndex
pAxItf

Type
INT
INT
POINTER

Description
These parameters correspond to the parameters of the same name of
the MC_AxAdsCommServer_BkPlcMc function block.

Here the address of the AXIS_REF_BkPlcMc [} 86] array is to be
transferred.

An incorrect specification at this point causes the PLC application
to crash inevitably through triggering of serious runtime errors
(Page Fault Exception).

 Outputs

VAR_OUTPUT
    PlcMcManOffline:    BOOL;
END_VAR

Name
PlcMcManOffline

Type
BOOL

Description
This parameter corresponds to the parameter of the same name of the
MC_AxAdsCommServer_BkPlcMc function block.

Crash of the PLC application

NOTICE

An incorrect specification at this point excludes some of the axes from the communication or results in a
crash of the PLC application by triggering serious runtime errors (Page Fault Exception)

Behavior of the function block

By cyclic call of this function block in the PLC application the transferred axes are connected to an internal
message buffer. The message buffer referenced when calling MC_AxUtiStandardInit_BkPlcMc [} 254] is
ignored when calling the MC_Communications_BkPlcMc function block. The messages from the internal
message buffer are cyclically transferred to the Windows Event Viewer by internally calling an instance of the
MC_AxRtLoggerSpool_BkPlcMc [} 261] function block. Furthermore the PLC application gets the character of
an ADS server, because internally an instance of the MC_AxAdsCommServer_BkPlcMc [} 279] function block
is called.

4.5.3

MC_AxAdsPtrArrCommServer_BkPlcMc

TF5810

Version: 1.8.3

281

MC_AxAdsPtrArrCommServer_BkPlcMcnFirstAxisIndex  INTnLastAxisIndex  INTpAxItfArr  Pointer To Pointer To AXIS_REF_BkPlcMcBOOL  PlcMcManOfflinePLCopen Motion Control

The function block gives the application the capacity to function as an ADS server. Calls function blocks of
type MC_AxAdsReadDecoder_BkPlcMc [} 283] and MC_AxAdsWriteDecoder_BkPlcMc [} 285] as required.
The ADS codes [} 345] are listed in the Knowledge Base.

For most applications an MC_AxAdsCommServer_BkPlcMc is adequate and preferable.

(MC_AxAdsCommServer_BkPlcMc [} 279])

 Inputs
VAR_INPUT
     nFirstAxisIndex:  INT;
     nLastAxisIndex:   INT;
END_VAR

Name
nFirstAxisIndex
nLastAxisIndex

Type
INT
INT

 Inputs/outputs

VAR_IN_OUT
     pAxItfArr:        POINTER TO DWORD;
END_VAR

Description
This parameter is used to specify the dimensioning of the
AXIS_REF_BkPlcMc [} 86] array.

An incorrect specification at this point excludes some
of the axes from the communication or results in a
crash of the PLC application by triggering serious
runtime errors (Page Fault Exception).

Name
pAxItfArr

Type
POINTER

Description
Here, the address of a variable of type ARRAY [ncnstFirstAxId..ncnstLastAxId]
OF POINTER TO AXIS_REF_BkPlcMc [} 86] should be transferred.

An incorrect specification at this point causes the PLC application to
crash inevitably through triggering of serious runtime errors (Page Fault
Exception).

 Outputs

VAR_OUTPUT
    PlcMcManOffline:    BOOL;
END_VAR

Name
PlcMcManOffline

Type
BOOL

Behavior of the function block

Description
This parameter corresponds to the parameter of the same name of the
MC_AxAdsCommServer_BkPlcMc function block.

Through cyclic calling of this function block in the PLC application, the application assumes the character of
an ADS server and responds to ADS read and ADS write-access like any other ADS server. This includes
the decoding of IdxGroup/IdxOffset addressing. Function blocks of type MC_AxAdsReadDecoder_BkPlcMc
[} 283] and MC_AxAdsWriteDecoder_BkPlcMc [} 285] are called as required.

This function block must not be used if the PLC application already is an ADS server.

282

Version: 1.8.3

TF5810

PLCopen Motion Control

In this case the function blocks of type MC_AxAdsReadDecoder_BkPlcMc [} 283] and
MC_AxAdsWriteDecoder_BkPlcMc [} 285] should be called from the existing ADS server function block of the
application.

A program example [} 321] #16 is available.

4.5.4

MC_AxAdsReadDecoder_BkPlcMc

Available from version 3.0

The function block decodes ADS read accesses. The ADS codes [} 345] are listed in the Knowledge Base.

 Inputs
VAR_INPUT
    nFirstAxisIndex:    INT;
    nLastAxisIndex:     INT;
    bReset:             BOOL;
    bValid:             BOOL;
    sNetId:             STRING(80);
    nPort:              UINT;
    nInvokeId:          UDINT;
    nIdxGroup:          UDINT;
    nIdxOffs:           UDINT;
    cbReadLen:          UDINT;
    pAxItf:             POINTER TO AXIS_REF_BkPlcMc:=0;
END_VAR

TF5810

Version: 1.8.3

283

MC_AxAdsReadDecoder_BkPlcMcnFirstAxisIndex  INTnLastAxisIndex  INTbReset  BOOLbValid  BOOLsNetId  STRING(80)nPort  UINTnInvokeId  UDINTnIdxGrp  UDINTnIdxOffs  UDINTcbReadLen  UDINTpAxItf  Pointer To AXIS_REF_BkPlcMc↔DeadManCount  Reference To UDINTBOOL  bClearBOOL  bPendingPLCopen Motion Control

Name
nFirstAxisIndex
nLastAxisIndex

Type
INT
INT

Description
This parameter is used to specify the dimensioning of the
AXIS_REF_BkPlcMc [} 86] array.

bReset
bValid
sNetId
nPort
nInvokeId
nIdxGroup
nIdxOffs
cbReadLen
pAxItf

BOOL
BOOL
STRING
UINT
UDINT
UDINT
UDINT
UDINT
POINTER

An incorrect specification at this point excludes some of the axes
from the communication or results in a crash of the PLC
application by triggering serious runtime errors (Page Fault
Exception).
The signals are used to co-ordinate the decoder with the ADS server.

These values are required in order to generate the ADS response.
They are supplied by an ADS server's ADS indication function block.

These values are required in order to decode the access. They are
supplied by an ADS server's ADS indication function block.

Here, the address of a variable or an array of variables of type
AXIS_REF_BkPlcMc [} 86] should be transferred.

 Inputs/outputs

VAR_IN_OUT
    DeadManCount:       UDINT;
END_VAR

Name
DeadManCount

Type
UDINT

Description

 Outputs

VAR_OUTPUT
    bClear:             BOOL;
    bPending:           BOOL;
END_VAR

Name
bClear
bPending

Type
BOOL
BOOL

Description
Indicates that an ADS access indicated with bValid should be acknowledged.
Indicates that an ADS access indicated with bValid is being processed.

Behavior of the function block

If, when the bValid signal is present, the function block indicates neither bClear nor bPending it has not
decoded the combination of nIdxGroup and nIdxOffs and has not generated a response. In such a case, the
ADS server (if there is one) must call another decoder, or must generate a response with the appropriate
error code.

284

Version: 1.8.3

TF5810

4.5.5

MC_AxAdsWriteDecoder_BkPlcMc

PLCopen Motion Control

Available from version 3.0

The function block decodes ADS write accesses. The ADS codes [} 345] are listed in the Knowledge Base.

 Inputs
VAR_INPUT
    nFirstAxisIndex:    INT;
    nLastAxisIndex:     INT;
    bReset:             BOOL;
    bValid:             BOOL;
    sNetId:             STRING(80);
    nPort:              UINT;
    nInvokeId:          UDINT;
    nIdxGroup:          UDINT;
    nIdxOffs:           UDINT;
    cbWriteLen:         UDINT;
    pWriteBuff:         DWORD;
    pAxItf:             POINTER TO AXIS_REF_BkPlcMc:=0;
END_VAR

Name
nFirstAxisIndex
nLastAxisIndex

Type
INT
INT

bReset
bValid
sNetId
nPort
nInvokeId
nIdxGroup
nIdxOffs
cbWriteLen
pWriteBuff
pAxItf

BOOL
BOOL
STRING
UINT
UDINT
UDINT
UDINT
UDINT
DWORD
POINTER

Description

The dimensions of the AXIS_REF_BkPlcMc [} 86] array must be specified
here.

An incorrect specification at this point excludes some of the axes
from the communication or results in a crash of the PLC
application by triggering serious runtime errors (Page Fault
Exception).
The signals are used to co-ordinate the decoder with the ADS server.

These values are required in order to generate the ADS response.
They are supplied by an ADS server's ADS indication function block.

These values are required in order to decode the access. They are
supplied by an ADS server's ADS indication function block.

Here, the address of a variable or an array of variables of type
AXIS_REF_BkPlcMc [} 86] should be transferred.

TF5810

Version: 1.8.3

285

MC_AxAdsWriteDecoder_BkPlcMcnFirstAxisIndex  INTnLastAxisIndex  INTbReset  BOOLbValid  BOOLsNetId  STRING(80)nPort  UINTnInvokeId  UDINTnIdxGrp  UDINTnIdxOffs  UDINTcbWriteLen  UDINTpWriteBuff  XWORDpAxItf  Pointer To AXIS_REF_BkPlcMc↔DeadManCount  Reference To UDINTBOOL  bClearBOOL  bPendingBOOL  PlcMcManOfflinePLCopen Motion Control

 Inputs/outputs

VAR_IN_OUT
    DeadManCount:       UDINT;
END_VAR

Name
DeadManCount

Type
UDINT

 Outputs

VAR_OUTPUT
    bClear:             BOOL;
    bPending:           BOOL;
    PlcMcManOffline:    BOOL;
END_VAR

Description
Counter for function block calls without ADS indication

Name
bClear

bPending
PlcMcManOffline

Type
BOOL

BOOL
BOOL

Description
Indicates that an ADS access indicated with bValid should be
acknowledged.
Indicates that an ADS access indicated with bValid is being processed.
Indicates that the connection to the PlcMcManager is not available.

Behavior of the function block

If the function block signals neither bClear nor bPending when the bValid signal is present, it has not
decoded the combination of nIdxGroup and nIdxOffs and no response has been generated. In such a case,
the ADS server (if there is one) must call another decoder, or must generate a response with the appropriate
error code.

4.5.6

MC_AxParamAuxLabelsLoad_BkPlcMc

Available from version 3.0

The function block loads the label texts for the customer-specific axis parameters from a file. These texts can
be generated with a simple text editor such as Microsoft Notepad.

NOTICE

The file must be structured according to the rules specified below. Otherwise, significant problems may
occur, including system crash.

This function block is generally not called directly by the application. If possible, a function block of type
MC_AxUtiStandardInit_BkPlcMc [} 254] should be used, which uses a function block of type
MC_AxParamAuxLabelsLoad_BkPlcMc.

 Inputs
VAR_INPUT
    Execute:        BOOL;
END_VAR

Name
Execute

Type
BOOL

Description
The loading process is initiated by a rising edge at this input.

286

Version: 1.8.3

TF5810

MC_AxParamAuxLabelsLoad_BkPlcMcExecute  BOOL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  DoneBOOL  ErrorUDINT  ErrorIdPLCopen Motion Control

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Done:           BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
Done
Error
ErrorID

Type
BOOL
BOOL
UDINT

Description
Successful loading of the parameters is indicated here.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If one of the pointers has not been initialized the system responds with Error and

ErrorID:=dwTcHydErrCdPtrPlcMc or dwTcHydErrCdPtrMcPlc.

The loading process begins if these checks are carried out without problems.

A falling edge at Execute clears all the pending output signals. If Execute is set to FALSE while the loading
process is still active, the process that had started continues unaffected. The signals provided at the end of
the operation (Error, ErrorID, Done) are made available for one cycle.

The number of rows in the file must match the number specified in the global constants of the library
as iTcHydfCustDataMaxIdx (currently: 20). The maximum number of characters in each row is 20
(included spaces, without line breaks).

4.5.7

MC_AxParamLoad_BkPlcMc

Available from version 3.0

The function block loads the parameters for an axis from a file. A function block of type
MC_AxParamSave_BkPlcMc [} 288] must be used to generate a compatible parameter file.

This function block is generally not called directly by the application. If possible, a function block of type
MC_AxUtiStandardInit_BkPlcMc [} 254] should be used, which uses a function block of type
MC_AxParamLoad_BkPlcMc.

TF5810

Version: 1.8.3

287

MC_AxParamLoad_BkPlcMcExecute  BOOL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  DoneBOOL  ErrorUDINT  ErrorIdPLCopen Motion Control

 Inputs
VAR_INPUT
    Execute:        BOOL;
END_VAR

Name
Execute

Type
BOOL

Description
The loading process is initiated by a rising edge at this input.

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Done:           BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
Done
Error
ErrorID

Type
BOOL
BOOL
UDINT

Description
Successful loading of the parameters is indicated here.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If the file cannot be opened for reading, the system responds with Error and

ErrorID:=dwTcHydErrCdPtrPlcMc or dwTcHydErrCdPtrMcPlc.

The loading process begins if these checks are carried out without problems. The file version is determined,
and any parameters that are not specified by the file are replaced with neutral default values. If the file
contains parameters that are not used or no longer used, these are ignored.

A falling edge at Execute clears all the pending output signals. If Execute is set to FALSE while the loading
process is still active, the process that had started continues unaffected. The signals provided at the end of
the operation (Error, ErrorID, Done) are made available for one cycle.

4.5.8

MC_AxParamSave_BkPlcMc

Available from version 3.0

The function block writes the parameters for an axis into a file. A function block of type
MC_AxParamLoad_BkPlcMc [} 287] must be used to read the file.

288

Version: 1.8.3

TF5810

MC_AxParamSave_BkPlcMcExecute  BOOL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  DoneBOOL  ErrorUDINT  ErrorIdPLCopen Motion Control

 Inputs
VAR_INPUT
    Execute:        BOOL;
END_VAR

Name
Execute

Type
BOOL

Description
The writing process is initiated by a rising edge at this input.

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Done:           BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
Done
Error
ErrorID

Type
BOOL
BOOL
UDINT

Description
Successful writing of the parameters is indicated here.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If the file cannot be opened for writing, the system responds with Error and

ErrorID:=dwTcHydErrCdPtrPlcMc or dwTcHydErrCdPtrMcPlc.

The writing process begins if these checks are carried out without problems. The versions of the saved
parameters are logged.

A falling edge at Execute clears all the pending output signals. If Execute is set to FALSE while the writing
process is still active, the process that had started continues unaffected. The signals provided at the end of
the operation (Error, ErrorID, Done) are made available for one cycle.

4.5.9

MC_AxUtiReadCoeDriveTerm_BkPlcMc

Available from version 3.0

TF5810

Version: 1.8.3

289

MC_AxUtiReadCoeDriveTerm_BkPlcMcExecute  BOOLPdata  Pointer To BYTEByteCount  BYTEIndex  WORDSubindex  BYTE↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

The function block reads the contents of a register from the EL terminal, which is used as drive interface for
the axis.

 Inputs
VAR_INPUT
    Execute:        BOOL;
    Pdata:          POINTER TO BYTE:=0;
    ByteCount:      BYTE:=0;
    Index:          WORD:=0;
    Subindex:       BYTE:=0;
END_VAR

Name
Execute
Pdata

Type
BOOL
POINTER

ByteCount
Index
Subindex

BYTE
WORD
BYTE

Description
A rising edge at this input starts the read process.
Here, the address of the variable is specified, in which the read value is to be
output.
Here, the size of the variable is specified in bytes.
Here, the addressing of parameter in the terminal is specified.

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF _BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    Done:           BOOL;
    CommandAborted: BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
Busy
Done
CommandAborted
Error
ErrorID

Type
BOOL
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
Successful loading of the parameter is indicated here.
Indicates abortion of the read operation.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If the axis is enabled for operation, the system responds with Error and

ErrorID:=dwTcHydErrCdNotReady.

• If Index or Subindex are out of range the system responds with Error and

ErrorID:=dwTcHydErrCdTblIllegalIndex.

• If ByteCount or Pdata are out of range the system responds with Error and

ErrorID:=dwTcHydErrCdTblIllegalIndex.

• If an I/O module, which does not support parameter communication, is set as nDrive_Type in the axis

parameters, the system responds with Error and ErrorID:=dwTcHydErrCdNotCompatible.

290

Version: 1.8.3

TF5810

PLCopen Motion Control

• If problems occur during the ADS communication with the terminal, the corresponding ADS error code

is returned as ErrorID, and Error is indicated. The following codes [} 339] may occur:

◦ 16#0006 = 6 = The port number of the ADS address used is invalid: Check mapping of the

InfoData element of the terminal!

◦ 16#0007 = 7 = The AmsNetID of the ADS address used is invalid: Check mapping of the InfoData

element of the terminal!

◦ 16#0702 = 1794 = dwTcHydAdsErrInvalidIdxGroup = The terminal does not support the CoE

protocol.

◦ 16#0703 = 1795 = dwTcHydAdsErrInvalidIdxOffset = The address in index and subindex is not

supported in the terminal.

◦ 16#0745 = 1861 = dwTcHydAdsErrTimeout = Timeout.

A falling edge at Execute clears all the pending output signals. If Execute is set to FALSE while the loading
process is still active, the process that had started continues unaffected. The signals provided at the end of
the operation (Done, CommandAborted, Error, ErrorID) are made available for one cycle.

4.5.10

MC_AxUtiReadCoeEncTerm_BkPlcMc

Available from version 3.0

The function block reads the contents of a register from the EL terminal, which is used as encoder interface
for the axis.

 Inputs
VAR_INPUT
    Execute:        BOOL;
    Pdata:          POINTER TO BYTE:=0;
    ByteCount:      BYTE:=0;
    Index:          WORD:=0;
    Subindex:       BYTE:=0;
END_VAR

Name
Execute
Pdata

Type
BOOL
POINTER

ByteCount
Index
Subindex

BYTE
WORD
BYTE

Description
A rising edge at this input starts the read process.
Here, the address of the variable is specified, in which the read value is to be
output.
Here, the size of the variable is specified in bytes.
Here, the addressing of parameter in the terminal is specified.

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

TF5810

Version: 1.8.3

291

MC_AxUtiReadCoeEncTerm_BkPlcMcExecute  BOOLPdata  Pointer To BYTEByteCount  BYTEIndex  WORDSubindex  BYTE↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDDescription

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

PLCopen Motion Control

Name
Axis

Type
AXIS_REF_BkPlcMc

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    Done:           BOOL;
    CommandAborted: BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
Busy
Done
CommandAborted
Error
ErrorID

Type
BOOL
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
Successful loading of the parameter is indicated here.
Indicates abortion of the read operation.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If the axis is enabled for operation, the system responds with Error and

ErrorID:=dwTcHydErrCdNotReady.

• If Index or Subindex are out of range the system responds with Error and

ErrorID:=dwTcHydErrCdTblIllegalIndex.

• If ByteCount or Pdata are out of range the system responds with Error and

ErrorID:=dwTcHydErrCdTblIllegalIndex.

• If an I/O module, which does not support parameter communication, is set as nEncoder_Type in the

axis parameters, the system responds with Error and ErrorID:=dwTcHydErrCdNotCompatible.

• If problems occur during the ADS communication with the terminal, the corresponding ADS error code

is returned as ErrorID, and Error is indicated. The following codes [} 339] may occur:

◦ 16#0006 = 6 = The port number of the ADS address used is invalid: Check mapping of the

InfoData element of the terminal!

◦ 16#0007 = 7 = The AmsNetID of the ADS address used is invalid: Check mapping of the InfoData

element of the terminal!

◦ 16#0702 = 1794 = dwTcHydAdsErrInvalidIdxGroup = The terminal does not support the CoE

protocol.

◦ 16#0703 = 1795 = dwTcHydAdsErrInvalidIdxOffset = The address in index and subindex is not

supported in the terminal.

◦ 16#0745 = 1861 = dwTcHydAdsErrTimeout = Timeout.

A falling edge at Execute clears all the pending output signals. If Execute is set to FALSE while the loading
process is still active, the process that had started continues unaffected. The signals provided at the end of
the operation (Done, CommandAborted, Error, ErrorID) are made available for one cycle.

292

Version: 1.8.3

TF5810

4.5.11

MC_AxUtiReadRegDriveTerm_BkPlcMc

PLCopen Motion Control

Available from version 3.0

The function block reads the contents of a register from the KL terminal, which is used as drive interface for
the axis.

 Inputs
VAR_INPUT
    Execute:        BOOL;
    Select:         INT;
END_VAR

Name
Execute
Select

Type
BOOL
INT

Description
A rising edge at this input starts the read process.
The register number should be transferred here.

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    RegData:        WORD;
    Busy:           BOOL;
    Done:           BOOL;
    CommandAborted: BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
RegData
Busy
Done
CommandAborted
Error
ErrorID

Type
WORD
BOOL
BOOL
BOOL
BOOL
UDINT

Description
The read value is output here.
Indicates that a command is being processed.
Successful loading of the parameter is indicated here.
Indicates abortion of the read operation.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

TF5810

Version: 1.8.3

293

MC_AxUtiReadRegDriveTerm_BkPlcMcExecute  BOOLSelect  INT↔Axis  Reference To AXIS_REF_BkPlcMcWORD  RegDataBOOL  BusyBOOL  DoneBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

• If one of the pointers ST_TcPlcDeviceInput [} 149] and ST_TcPlcDeviceOutput [} 153] is not initialized,

the system responds with Error and ErrorID:=dwTcHydErrCdPtrPlcMc.

• If the axis is enabled for operation, the system responds with Error and

ErrorID:=dwTcHydErrCdNotReady.

• If Select is out of the allowed range from 0 to 63, the system responds with Error and

ErrorID:=dwTcHydErrCdTblIllegalIndex.

• If an I/O module, which does not support parameter communication, is set as nDrive_Type in the axis

parameters, the system responds with Error and ErrorID:=dwTcHydErrCdNotCompatible.

If these checks could be performed without problem, the read operation is initiated.

A falling edge at Execute clears all the pending output signals. If Execute is set to FALSE while the loading
process is still active, the process that had started continues unaffected. The signals provided at the end of
the operation (RegData, Done, CommandAborted, Error, ErrorID, Done) are made available for one cycle.

The drive types iTcMc_DriveKL2521, iTcMc_DriveKL4032, iTcMc_DriveKL2531 and
iTcMc_DriveKL2541 support the parameter communication.

4.5.12

MC_AxUtiReadRegEncTerm_BkPlcMc

Available from version 3.0

The function block reads the contents of a register from the KL terminal, which is used as encoder interface
for the axis.

 Inputs
VAR_INPUT
    Execute:        BOOL;
    Select:         INT;
END_VAR

Name
Execute
Select

Type
BOOL
INT

Description
A rising edge at this input starts the read process.
The register number should be transferred here.

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

294

Version: 1.8.3

TF5810

MC_AxUtiReadRegEncTerm_BkPlcMcExecute  BOOLSelect  INT↔Axis  Reference To AXIS_REF_BkPlcMcWORD  RegDataBOOL  BusyBOOL  DoneBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

 Outputs

VAR_OUTPUT
    RegData:        WORD;
    Busy:           BOOL;
    Done:           BOOL;
    CommandAborted: BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
RegData
Busy
Done
CommandAborted
Error
ErrorID

Type
WORD
BOOL
BOOL
BOOL
BOOL
UDINT

Description
The read value is output here.
Indicates that a command is being processed.
Successful loading of the parameter is indicated here.
Indicates abortion of the read operation.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If one of the pointers ST_TcPlcDeviceInput [} 149] and ST_TcPlcDeviceOutput [} 153] is not initialized,

the system responds with Error and ErrorID:=dwTcHydErrCdPtrPlcMc.

• If the axis is enabled for operation, the system responds with Error and

ErrorID:=dwTcHydErrCdNotReady.

• If Select is out of the allowed range from 0 to 63, the system responds with Error and

ErrorID:=dwTcHydErrCdTblIllegalIndex.

• If an I/O module, which does not support parameter communication, is set as nEncoder_Type in the

axis parameters, the system responds with Error and ErrorID:=dwTcHydErrCdNotCompatible.

If these checks could be performed without problem, the read operation is initiated.

A falling edge at Execute clears all the pending output signals. If Execute is set to FALSE while the loading
process is still active, the process that had started continues unaffected. The signals provided at the end of
the operation (RegData, Done, CommandAborted, Error, ErrorID, Done) are made available for one cycle.

The drive types iTcMc_EncoderKL3002, iTcMc_EncoderKL3042, iTcMc_EncoderKL3062,
iTcMc_EncoderKL3162, iTcMc_EncoderKL5101, iTcMc_EncoderKL5111, iTcMc_EncoderKL2521,
iTcMc_EncoderKL2531 und iTcMc_EncoderKL2541 support parameter communication.

4.5.13

MC_AxUtiUpdateRegDriveTerm_BkPlcMc

Available from version 3.0.7

The function block writes a parameter set into the registers of a KL terminal. It uses
MC_AxUtiReadRegDriveTerm_BkPlcMc [} 293] and MC_AxUtiWriteRegDriveTerm_BkPlcMc [} 302] function
blocks for this purpose.

TF5810

Version: 1.8.3

295

MC_AxUtiUpdateRegDriveTerm_BkPlcMcExecute  BOOL↔Axis  Reference To AXIS_REF_BkPlcMc↔RegData  Reference To ST_TcPlcRegDataTableBOOL  DoneBOOL  BusyBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

 Inputs
VAR_INPUT
    Execute:        BOOL;
END_VAR

Name
Execute

Type
BOOL

Description
The writing process is initiated by a rising edge at this input.

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
    RegData:        ST_TcPlcRegDataTable;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

RegData

ST_TcPlcRegDataTable

 Outputs

VAR_OUTPUT
    Done:           BOOL;
    Busy:           BOOL;
    CommandAborted: BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc
[} 86] should be transferred.
Here, the address of parameter set should be specified,
whose content is to be written into the terminal.

Name
Done
Busy
CommandAborted
Error
ErrorID

Type
BOOL
BOOL
BOOL
BOOL
UDINT

Description
Indicates successful writing of the parameter.
Indicates that a command is being processed.
Indicates abortion of the read operation.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If one of the pointers ST_TcPlcDeviceInput [} 149] and ST_TcPlcDeviceOutput [} 153] is not initialized,

the system responds with Error and ErrorID:=dwTcHydErrCdPtrPlcMc.

• If the axis is enabled for operation, the system responds with Error and

ErrorID:=dwTcHydErrCdNotReady.

• If Select is out of the allowed range from 0 to 63, the system responds with Error and

ErrorID:=dwTcHydErrCdTblIllegalIndex.

• If an I/O module, which does not support parameter communication, is set as nDrive_Type in the axis

parameters, the system responds with Error and ErrorID:=dwTcHydErrCdNotCompatible.

• The value in ST_TcPlcRegDataTable [} 157].RegDataItem[...].Access determines how the element is

treated.

◦ 0: Element is ignored.

◦ 1: The register addressed through Select is read. Its contents are compared with RegData. If the

contents differ, the write operation is aborted with Error and ErrorID:=16#FFFFFFFF.

◦ 2: The register addressed through Select is read. Its contents are compared with RegData. If the
contents are not larger, the write operation is aborted with Error and ErrorID:=16#FFFFFFFF.

296

Version: 1.8.3

TF5810

PLCopen Motion Control

◦ 3: The register addressed through Select is read. Its contents are compared with RegData. If the
contents are not smaller, the write operation is aborted with Error and ErrorID:=16#FFFFFFFF.

◦ 4: The register addressed through Select is read. Its contents are compared with RegData. If the

contents are not larger or equal, the write operation is aborted with Error and
ErrorID:=16#FFFFFFFF.

◦ 5: The register addressed through Select is read. Its contents are compared with RegData. If the

contents are not smaller or equal, the write operation is aborted with Error and
ErrorID:=16#FFFFFFFF.

◦ 10: The register addressed through Select is written with RegData.

◦ Other values are currently ignored. Future versions of the library may support additional functions.

An empty element should therefore always be identified with 0.

A falling edge at Execute clears all the pending output signals. If Execute is set to FALSE while the writing
process is still active, the process that had started continues unaffected. The signals provided at the end of
the operation (Done, CommandAborted, Error, ErrorID, Done) are made available for one cycle.

4.5.14

MC_AxUtiUpdateRegEncTerm_BkPlcMc

Available from version 3.0

The function block writes a parameter set into the registers of a KL terminal. It uses
MC_AxUtiReadRegEncTerm_BkPlcMc [} 294] and MC_AxUtiWriteRegEncTerm_BkPlcMc [} 303] function blocks
for this purpose.

 Inputs
VAR_INPUT
    Execute:        BOOL;
END_VAR

Name
Execute

Type
BOOL

Description
The writing process is initiated by a rising edge at this input.

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
    RegData:        ST_TcPlcRegDataTable;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

RegData

ST_TcPlcRegDataTable

 Outputs

VAR_OUTPUT
    Done:           BOOL;
    Busy:           BOOL;

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc
[} 86] should be transferred.
Here, the address of parameter set should be specified,
whose content is to be written into the terminal.

TF5810

Version: 1.8.3

297

MC_AxUtiUpdateRegEncTerm_BkPlcMcExecute  BOOL↔Axis  Reference To AXIS_REF_BkPlcMc↔RegData  Reference To ST_TcPlcRegDataTableBOOL  DoneBOOL  BusyBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

    CommandAborted: BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
Done
Busy
CommandAborted
Error
ErrorID

Type
BOOL
BOOL
BOOL
BOOL
UDINT

Description
Indicates successful writing of the parameter.
Indicates that a command is being processed.
Indicates abortion of the read operation.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If one of the pointers ST_TcPlcDeviceInput [} 149] and ST_TcPlcDeviceOutput [} 153] is not initialized,

the system responds with Error and ErrorID:=dwTcHydErrCdPtrPlcMc.

• If the axis is enabled for operation, the system responds with Error and

ErrorID:=dwTcHydErrCdNotReady.

• If Select is out of the allowed range from 0 to 63, the system responds with Error and

ErrorID:=dwTcHydErrCdTblIllegalIndex.

• If an I/O module, which does not support parameter communication, is set as nDrive_Type in the axis

parameters, the system responds with Error and ErrorID:=dwTcHydErrCdNotCompatible.

• The value in ST_TcPlcRegDataTable [} 157].RegDataItem[...].Access determines how the element is

treated.

◦ 0: Element is ignored.

◦ 1: The register addressed through Select is read. Its contents are compared with RegData. If the

contents differ, the write operation is aborted with Error and ErrorID:=16#FFFFFFFF.

◦ 2: The register addressed through Select is read. Its contents are compared with RegData. If the
contents are not larger, the write operation is aborted with Error and ErrorID:=16#FFFFFFFF.

◦ 3: The register addressed through Select is read. Its contents are compared with RegData. If the
contents are not smaller, the write operation is aborted with Error and ErrorID:=16#FFFFFFFF.

◦ 4: The register addressed through Select is read. Its contents are compared with RegData. If the

contents are not larger or equal, the write operation is aborted with Error and
ErrorID:=16#FFFFFFFF.

◦ 5: The register addressed through Select is read. Its contents are compared with RegData. If the

contents are not smaller or equal, the write operation is aborted with Error and
ErrorID:=16#FFFFFFFF.

◦ 10: The register addressed through Select is written with RegData.

◦ Other values are currently ignored. Future versions of the library may support additional functions.

An empty element should therefore always be identified with 0.

A falling edge at Execute clears all the pending output signals. If Execute is set to FALSE while the writing
process is still active, the process that had started continues unaffected. The signals provided at the end of
the operation (Done, CommandAborted, Error, ErrorID, Done) are made available for one cycle.

298

Version: 1.8.3

TF5810

4.5.15

MC_AxUtiWriteCoeDriveTerm_BkPlcMc

PLCopen Motion Control

Available from version 3.0

The function block writes the contents of a register of the EL terminal, which is used as drive interface for the
axis.

 Inputs
VAR_INPUT
    Execute:        BOOL;
    Pdata:          POINTER TO BYTE:=0;
    ByteCount:      BYTE:=0;
    Index:          WORD:=0;
    Subindex:       BYTE:=0;
END_VAR

Name
Execute
Pdata

Type
BOOL
POINTER

ByteCount
Index
Subindex

BYTE
WORD
BYTE

Description
The writing process is initiated by a rising edge at this input.
The address of the variable whose content is to be written to the terminal
must be specified here.
Here, the size of the variable is specified in bytes.
Here, the addressing of parameter in the terminal is specified.

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    Done:           BOOL;
    CommandAborted: BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
Busy
Done
CommandAborted
Error
ErrorID

Type
BOOL
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
Indicates successful writing of the parameter.
Indicates abortion of the read operation.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

TF5810

Version: 1.8.3

299

MC_AxUtiWriteCoeDriveTerm_BkPlcMcExecute  BOOLPdata  Pointer To BYTEByteCount  BYTEIndex  WORDSubindex  BYTE↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If the axis is enabled for operation, the system responds with Error and

ErrorID:=dwTcHydErrCdNotReady.

• If Index or Subindex are out of range the system responds with Error and

ErrorID:=dwTcHydErrCdTblIllegalIndex.

• If ByteCount or Pdata are out of range the system responds with Error and

ErrorID:=dwTcHydErrCdTblIllegalIndex.

• If an I/O module, which does not support parameter communication, is set as nDrive_Type in the axis

parameters, the system responds with Error and ErrorID:=dwTcHydErrCdNotCompatible.

• If problems occur during the ADS communication with the terminal, the corresponding ADS error code

is returned as ErrorID, and Error is indicated. The following codes [} 339] may occur:

◦ 16#0006 = 6 = The port number of the ADS address used is invalid: Check mapping of the

InfoData element of the terminal!

◦ 16#0007 = 7 = The AmsNetID of the ADS address used is invalid: Check mapping of the InfoData

element of the terminal!

◦ 16#0702 = 1794 = dwTcHydAdsErrInvalidIdxGroup = The terminal does not support the CoE

protocol.

◦ 16#0703 = 1795 = dwTcHydAdsErrInvalidIdxOffset = The address in index and subindex is not

supported in the terminal.

◦ 16#0745 = 1861 = dwTcHydAdsErrTimeout = Timeout.

A falling edge at Execute clears all the pending output signals. If Execute is set to FALSE while the writing
process is still active, the process that had started continues unaffected. The signals provided at the end of
the operation (Done, CommandAborted, Error, ErrorID) are made available for one cycle.

4.5.16

MC_AxUtiWriteCoeEncTerm_BkPlcMc

Available from version 3.0

The function block writes the contents of a register of the EL terminal, which is used as encoder interface for
the axis.

 Inputs
VAR_INPUT
    Execute:        BOOL;
    Pdata:          POINTER TO BYTE:=0;
    ByteCount:      BYTE:=0;
    Index:          WORD:=0;
    Subindex:       BYTE:=0;
END_VAR

300

Version: 1.8.3

TF5810

MC_AxUtiWriteCoeEncTerm_BkPlcMcExecute  BOOLPdata  Pointer To BYTEByteCount  BYTEIndex  WORDSubindex  BYTE↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

Name
Execute
Pdata

Type
BOOL
POINTER

ByteCount
Index
Subindex

BYTE
WORD
BYTE

Description
The writing process is initiated by a rising edge at this input.
The address of the variable whose content is to be written to the terminal
must be specified here.
Here, the size of the variable is specified in bytes.
Here, the addressing of parameter in the terminal is specified.

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    Done:           BOOL;
    CommandAborted: BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
Busy
Done
CommandAborted
Error
ErrorID

Type
BOOL
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
Indicates successful writing of the parameter.
Indicates abortion of the read operation.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If the axis is enabled for operation, the system responds with Error and

ErrorID:=dwTcHydErrCdNotReady.

• If Index or Subindex are out of range the system responds with Error and

ErrorID:=dwTcHydErrCdTblIllegalIndex.

• If ByteCount or Pdata are out of range the system responds with Error and

ErrorID:=dwTcHydErrCdTblIllegalIndex.

• If an I/O module, which does not support parameter communication, is set as nEncoder_Type in the

axis parameters, the system responds with Error and ErrorID:=dwTcHydErrCdNotCompatible.

• If problems occur during the ADS communication with the terminal, the corresponding ADS error code

is returned as ErrorID, and Error is indicated. The following codes [} 339] may occur:

◦ 16#0006 = 6 = The port number of the ADS address used is invalid: Check mapping of the

InfoData element of the terminal!

◦ 16#0007 = 7 = The AmsNetID of the ADS address used is invalid: Check mapping of the InfoData

element of the terminal!

◦ 16#0702 = 1794 = dwTcHydAdsErrInvalidIdxGroup = The terminal does not support the CoE

protocol.

◦ 16#0703 = 1795 = dwTcHydAdsErrInvalidIdxOffset = The address in index and subindex is not

supported in the terminal.

TF5810

Version: 1.8.3

301

PLCopen Motion Control

◦ 16#0745 = 1861 = dwTcHydAdsErrTimeout = Timeout.

A falling edge at Execute clears all the pending output signals. If Execute is set to FALSE while the writing
process is still active, the process that had started continues unaffected. The signals provided at the end of
the operation (Done, CommandAborted, Error, ErrorID) are made available for one cycle.

4.5.17

MC_AxUtiWriteRegDriveTerm_BkPlcMc

Available from version 3.0

The function block writes the contents of a register of the KL terminal, which is used as drive interface for the
axis.

 Inputs
VAR_INPUT
    Execute:        BOOL;
    Select:         INT;
    RegData:        WORD;
END_VAR

Name
Execute
Select
RegData

Type
BOOL
INT
WORD

Description
The writing process is initiated by a rising edge at this input.
The register number should be transferred here.
The value to be written should be transferred here.

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    Done:           BOOL;
    CommandAborted: BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
Busy
Done
CommandAborted
Error
ErrorID

Type
BOOL
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
Indicates successful writing of the parameter.
Indicates abortion of the read operation.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

302

Version: 1.8.3

TF5810

MC_AxUtiWriteRegDriveTerm_BkPlcMcExecute  BOOLSelect  INTRegData  WORD↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If one of the pointers ST_TcPlcDeviceInput [} 149] and ST_TcPlcDeviceOutput [} 153] is not initialized,

the system responds with Error and ErrorID:=dwTcHydErrCdPtrPlcMc.

• If the axis is enabled for operation, the system responds with Error and

ErrorID:=dwTcHydErrCdNotReady.

• If Select is out of the allowed range from 0 to 63, the system responds with Error and

ErrorID:=dwTcHydErrCdTblIllegalIndex.

• If an I/O module, which does not support parameter communication, is set as nDrive_Type in the axis

parameters, the system responds with Error and ErrorID:=dwTcHydErrCdNotCompatible.

The writing process begins if these checks are carried out without problems.

A falling edge at Execute clears all the pending output signals. If Execute is set to FALSE while the writing
process is still active, the process that had started continues unaffected. The signals provided at the end of
the operation (RegData, Done, CommandAborted, Error, ErrorID, Done) are made available for one cycle.

The drive types iTcMc_DriveKL2521, iTcMc_DriveKL4032, iTcMc_DriveKL2531 and
iTcMc_DriveKL2541 support the parameter communication.

4.5.18

MC_AxUtiWriteRegEncTerm_BkPlcMc

Available from version 3.0

The function block writes the contents of a register of the KL terminal, which is used as encoder interface for
the axis.

 Inputs
VAR_INPUT
    Execute:        BOOL;
    Select:         INT;
    RegData:        WORD;
END_VAR

Name
Execute
Select
RegData

Type
BOOL
INT
WORD

Description
The writing process is initiated by a rising edge at this input.
The register number should be transferred here.
The value to be written should be transferred here.

 Inputs/outputs

VAR_IN_OUT
    Axis:           AXIS_REF_BkPlcMc;
END_VAR

TF5810

Version: 1.8.3

303

MC_AxUtiWriteRegEncTerm_BkPlcMcExecute  BOOLSelect  INTRegData  WORD↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  BusyBOOL  DoneBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc
[} 86] should be transferred.

 Outputs

VAR_OUTPUT
    Busy:           BOOL;
    Done:           BOOL;
    CommandAborted: BOOL;
    Error:          BOOL;
    ErrorID:        UDINT;
END_VAR

Name
Busy
Done
CommandAborted
Error
ErrorID

Type
BOOL
BOOL
BOOL
BOOL
UDINT

Description
Indicates that a command is being processed.
Indicates successful writing of the parameter.
Indicates abortion of the read operation.
The occurrence of an error is indicated here.
An encoded indication of the cause of the error is provided here.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• If one of the pointers ST_TcPlcDeviceInput [} 149] and ST_TcPlcDeviceOutput [} 153] is not initialized,

the system responds with Error and ErrorID:=dwTcHydErrCdPtrPlcMc.

• If the axis is enabled for operation, the system responds with Error and

ErrorID:=dwTcHydErrCdNotReady.

• If Select is out of the allowed range from 0 to 63, the system responds with Error and

ErrorID:=dwTcHydErrCdTblIllegalIndex.

• If an I/O module, which does not support parameter communication, is set as nEncoder_Type in the

axis parameters, the system responds with Error and ErrorID:=dwTcHydErrCdNotCompatible.

The writing process begins if these checks are carried out without problems.

A falling edge at Execute clears all the pending output signals. If Execute is set to FALSE while the writing
process is still active, the process that had started continues unaffected. The signals provided at the end of
the operation (RegData, Done, CommandAborted, Error, ErrorID, Done) are made available for one cycle.

The drive types iTcMc_EncoderKL3002, iTcMc_EncoderKL3042, iTcMc_EncoderKL3062,
iTcMc_EncoderKL3162, iTcMc_EncoderKL5101, iTcMc_EncoderKL5111, iTcMc_EncoderKL2521,
iTcMc_EncoderKL2531 und iTcMc_EncoderKL2541 support parameter communication.

304

Version: 1.8.3

TF5810

PLCopen Motion Control

4.6

Part 5 Homing

4.6.1

FinalizingFunctions

4.6.1.1

MC_AbortHoming_BkPlcMc

The function block is used to cancel a referencing process.

 Inputs
VAR_INPUT
    Execute:             BOOL;
END_VAR

Name
Execute

Type
BOOL

Description
A rising edge at this input starts the abort.

 Inputs/outputs

VAR_IN_OUT
    Axis:       AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Done:               BOOL;
    Busy:               BOOL;
    Active:             BOOL;
    CommandAborted:     BOOL;
    Error:              BOOL;
    ErrorID:            UDINT;
END_VAR

Name
Done
Busy
Active
CommandAborted
Error
ErrorID

Type
BOOL
BOOL
BOOL
BOOL
BOOL
UDINT

Description
Successful processing is indicated here
Indicates that a command is being processed.
Readiness for operation is indicated here.
Here an abort of the command is indicated.
The occurrence of an error is indicated here.
An encoded error message is provided here.

TF5810

Version: 1.8.3

305

MC_AbortHoming_BkPlcMcExecute  BOOL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  DoneBOOL  BusyBOOL  ActiveBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface whether an active
movement is executed. If the axis is not in an active movement, referencing is aborted directly. If the axis is
in an active movement, this movement is stopped via a MC_Stop_BkPlcMc [} 82]. If the stop is successful,
the function block reports Done. If an error occurs during the stop, this error is indicated via Error and
ErrorId.

A falling edge at Execute clears all the pending output signals. If, while the movement is still active, Execute
is set to FALSE, execution of the command that had started continues unaffected

4.6.1.2

MC_FinishHoming_BkPlcMc

The function block is used to cancel a referencing process.

 Inputs
VAR_INPUT
    Execute:          BOOL;
    Distance:         LREAL;
    Velocity:         LREAL;
    Acceleration:     LREAL;
    Deceleration:     LREAL;
    Jerk:             LREAL;
    BufferMode:       MC_BufferMode_BkPlcMc:=Aborting_BkPlcMc;
END_VAR

Name
Execute

Type
BOOL

Distance

LREAL

Velocity

LREAL

Acceleration

LREAL

Deceleration

LREAL

Jerk

LREAL

BufferMode

MC_BufferMode_BkPlc
Mc

Description
A rising edge at this input starts the movement and terminates
the referencing.
[mm] The distance to the target position of the movement in
actual value units of the axis.
[mm/s] The required motion velocity in actual value units of the
axis per second.
[mm/s2] The required acceleration in actual value units of the axis
per square second. If this parameter is 0.0, it is replaced by a
default value from the axis parameters.
[mm/s2] The required deceleration in actual value units of the
axis per square second. If this parameter is 0.0, it is replaced by
a default value from the axis parameters.
[mm/s3] The required jerk in actual value units of the axis per
square second. If this parameter is 0.0, it is replaced by a default
value from the axis parameters.
reserved

306

Version: 1.8.3

TF5810

MC_FinishHoming_BkPlcMcExecute  BOOLDistance  LREALVelocity  LREALAcceleration  LREALDeceleration  LREALJerk  LREALBufferMode  MC_BufferMode_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  DoneBOOL  BusyBOOL  ActiveBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

 Inputs/outputs

VAR_IN_OUT
    Axis:               AXIS_REF_BkPlcMc;
END_VAR

Name
Axis sLog

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86]
should be transferred.

 Outputs

VAR_OUTPUT
    Done:               BOOL;
    Busy:               BOOL;
    Active:             BOOL;
    CommandAborted:     BOOL;
    Error:              BOOL;
    ErrorID:            UDINT;
END_VAR

Name
Done
Busy
Active
CommandAborted
Error
ErrorID

Type
BOOL
BOOL
BOOL
BOOL
BOOL
UDINT

Description
Successful processing is indicated here
Indicates that a command is being processed.
Readiness for operation is indicated here.
Here an abort of the command is indicated.
The occurrence of an error is indicated here.
An encoded error message is provided here.

Behavior of the function block

On a rising edge at Execute the parameters Distance, Velocity, Acceleration and Deceleration are
transferred to the MC_MoveRelative_BkPlcMc [} 77] function block. After checking the transferred
parameters, the movement is executed.

If the motion algorithm reports an error code while the movement is being executed, the system responds
with Error and ErrorID:=the motion algorithm's error code. If completion of the movement is prevented by
the activity of another function block, the system responds with CommandAborted. If the motion algorithm
achieves the target conditions for the axis, the system responds with Done.

A falling edge at Execute clears all the pending output signals. If, while the movement is still active, Execute
is set to FALSE, execution of the movement that had started continues unaffected. The signals provided at
the end of the movement (Error, ErrorID, CommandAborted, Done) are made available for one cycle.

The axis is in the state McState_Homing [} 103] during the movement, at the end the state changes to
McState_Standstill [} 103].

TF5810

Version: 1.8.3

307

PLCopen Motion Control

4.6.2

StepFunctions

4.6.2.1

MC_StepAbsoluteSwitch_BkPlcMc

The function block is used for referencing via a limit switch. The function block triggers a position setting
internally after the cam is found.

 Inputs
VAR_INPUT
    Execute:              BOOL;
    Direction:            MC_Direction_BkPlcMc;
    SwitchMode:           MC_SwitchMode_BkPlcMc;
    ReferenceSignal:      MC_Ref_Signal_Ref_BkPlcMc;
    Velocity:             LREAL;
    Acceleration:         LREAL;
    Deceleration:         LREAL;
    Jerk:                 LREAL;
    SetPosition:          LREAL;
    TorqueLimit:          LREAL;
    TimeLimit:            TIME;
    DistanceLimit:        LREAL;
    BufferMode:           MC_BufferMode_BkPlcMc:=Aborting_BkPlcMc;
END_VAR

308

Version: 1.8.3

TF5810

MC_StepAbsoluteSwitch_BkPlcMcExecute  BOOLDirection  MC_Direction_BkPlcMcSwitchMode  MC_SwitchMode_BkPlcMcReferenceSignal  MC_Ref_Signal_Ref_BkPlcMcVelocity  LREALAcceleration  LREALDeceleration  LREALJerk  LREALSetPosition  LREALTorqueLimit  LREALTimeLimit  TIMEDistanceLimit  LREALBufferMode  MC_BufferMode_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  DoneBOOL  BusyBOOL  ActiveBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

Description
The homing is initiated by a rising edge at this input.

The direction is specified via MC_Direction_BkPlcMc
[} 124].
The type of signal detection is specified via
MC_SwitchMode_BkPlcMc [} 124].
The signal state of the cam is communicated via
MC_Ref_Signal_Ref_BkPlcMc [} 165].
[mm/s] The required motion velocity in actual value units
of the axis per second.
[mm/s2] The required acceleration in actual value units of
the axis per square second. If this parameter is 0.0, it is
replaced by a default value from the axis parameters.
[mm/s2] The required deceleration in actual value units of
the axis per square second. If this parameter is 0.0, it is
replaced by a default value from the axis parameters.
[mm/s3] The required jerk in actual value units of the axis
per square second. If this parameter is 0.0, it is replaced
by a default value from the axis parameters.
Required position on the referencing cam.
reserved
After this time, the referencing is canceled with error. At
zero, time monitoring is disabled.
After this distance, the referencing will be aborted with an
error. At zero, the distance monitoring is disabled.
reserved

Name
Execute
Direction

Type
BOOL
MC_Direction_BkPlcMc

SwitchMode

ReferenceSignal

MC_SwitchMode_BkPlcM
c

MC_Ref_Signal_Ref_BkP
lcMc

Velocity

LREAL

Acceleration

LREAL

Deceleration

LREAL

Jerk

LREAL

SetPosition
TorqueLimit
TimeLimit

LREAL
LREAL
TIME

DistanceLimit

LREAL

BufferMode

MC_BufferMode_BkPlcM
c

 Inputs/outputs

VAR_IN_OUT
    Axis:               AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Done:               BOOL;
    Busy:               BOOL;
    Active:             BOOL;
    CommandAborted:     BOOL;
    Error:              BOOL;
    ErrorID:            UDINT;
END_VAR

Name
Done
Busy
Active
CommandAborte
d
Error
ErrorID

Type
BOOL
BOOL
BOOL
BOOL

Description
Successful processing is indicated here
Indicates that a command is being processed.
Readiness for operation is indicated here.
Here an abort of the command is indicated.

BOOL
UDINT

The occurrence of an error is indicated here.
An encoded error message is provided here.

TF5810

Version: 1.8.3

309

PLCopen Motion Control

Behavior of the function block

On a rising edge at Execute the parameters Direction, SwitchMode, ReferenceSignal, Velocity,
Acceleration and Deceleration are transferred to the MC_StepAbsoluteSwitchDetection_BkPlcMc [} 310]
function block. If the internal function block MC_StepAbsoluteSwitchDetection_BkPlcMc is successfully
processed, the determined position is set accordingly via MC_SetPosition_BkPlcMc [} 43].

During processing, the function block reports Busy and Active. After successful position setting, Done is
reported back. If a subordinate function block reports an error, this is reported via Error and ErrorID.

4.6.2.2

MC_StepAbsoluteSwitchDetection_BkPlcMc

The function block is used for referencing via a limit switch. The function block outputs the position of the
cam to the outside via RecordedPosition. No position is set.

 Inputs
VAR_INPUT
    Execute:              BOOL;
    Direction:            MC_Direction_BkPlcMc;
    SwitchMode:           MC_SwitchMode_BkPlcMc;
    ReferenceSignal:      MC_Ref_Signal_Ref_BkPlcMc;
    Velocity:             LREAL;
    Acceleration:         LREAL;
    Deceleration:         LREAL;
    Jerk:                 LREAL;
    SetPosition:          LREAL;
    TorqueLimit:          LREAL;
    TimeLimit:            TIME;
    DistanceLimit:        LREAL;
    BufferMode:           MC_BufferMode_BkPlcMc:=Aborting_BkPlcMc;
END_VAR

310

Version: 1.8.3

TF5810

MC_StepAbsoluteSwitchDetection_BkPlcMcExecute  BOOLDirection  MC_Direction_BkPlcMcSwitchMode  MC_SwitchMode_BkPlcMcReferenceSignal  MC_Ref_Signal_Ref_BkPlcMcVelocity  LREALAcceleration  LREALDeceleration  LREALJerk  LREALSetPosition  LREALTorqueLimit  LREALTimeLimit  TIMEDistanceLimit  LREALBufferMode  MC_BufferMode_BkPlcMcDetectionVelocityLimit  LREALDetectionVelocityTime  TIME↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  DoneBOOL  BusyBOOL  ActiveBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDLREAL  RecordedPositionPLCopen Motion Control

Description
The homing is initiated by a rising edge at this input.

The direction is specified via MC_Direction_BkPlcMc [} 124].

The type of signal detection is specified via
MC_SwitchMode_BkPlcMc [} 124].
The signal state of the cam is communicated via
MC_Ref_Signal_Ref_BkPlcMc [} 165].
[mm/s] The required motion velocity in actual value units of the
axis per second.
[mm/s2] The required acceleration in actual value units of the
axis per square second. If this parameter is 0.0, it is replaced
by a default value from the axis parameters.
[mm/s2] The required deceleration in actual value units of the
axis per square second. If this parameter is 0.0, it is replaced
by a default value from the axis parameters.
[mm/s3] The required jerk in actual value units of the axis per
square second. If this parameter is 0.0, it is replaced by a
default value from the axis parameters.
Required position on the referencing cam.
reserved
After this time, the referencing is canceled with error. At zero,
time monitoring is disabled.
After this distance, the referencing will be aborted with an error.
At zero, the distance monitoring is disabled.

Name
Execute
Direction

SwitchMode

ReferenceSignal

Type
BOOL
MC_Direction_BkPlc
Mc
MC_SwitchMode_B
kPlcMc

MC_Ref_Signal_Ref
_BkPlcMc

Velocity

LREAL

Acceleration

LREAL

Deceleration

LREAL

Jerk

LREAL

SetPosition
TorqueLimit
TimeLimit

LREAL
LREAL
TIME

DistanceLimit

LREAL

BufferMode

MC_BufferMode_Bk
PlcMc

 Inputs/outputs

VAR_IN_OUT
    Axis:               AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Done:               BOOL;
    Busy:               BOOL;
    Active:             BOOL;
    CommandAborted:     BOOL;
    Error:              BOOL;
    ErrorID:            UDINT;
    RecordedPosition:   LREAL;
END_VAR

Name
Done
Busy
Active
CommandAborted
Error
ErrorID
RecordedPosition

Type
BOOL
BOOL
BOOL
BOOL
BOOL
UDINT
LREAL

Description
Successful processing is indicated here
Indicates that a command is being processed.
Readiness for operation is indicated here.
Here an abort of the command is indicated.
The occurrence of an error is indicated here.
An encoded error message is provided here.
[mm] Determined position of the referencing cam.

TF5810

Version: 1.8.3

311

PLCopen Motion Control

Behavior of the function block

On a rising edge at Execute only MC_Positive_Direction_BkPlcMc, MC_Negative_Direction_BkPlcMc,
MC_SwitchPositive_Direction_BkPlcMc, MC_SwitchNegative_Direction_BkPlcMc are accepted at the
parameter Direction. The parameters Velocity, Acceleration, Deceleration and Jerk are transferred to
MC_MoveVelocity_BkPlcMc [} 79]. After the cam has been detected, the position is communicated via
RecordedPosition and a MC_Halt_BkPlcMc [} 71] aborts the movement.

During processing, the function block reports Busy and Active. After successful processing Done is
reported back. If a subordinate function block reports an error, this is reported via Error and ErrorID.

4.6.2.3

MC_StepBlock_BkPlcMc

The function block is used for referencing via a fixed stop. The function block triggers a position setting
internally after the cam is found.

 Inputs
VAR_INPUT
    Execute:                BOOL;
    Direction:              MC_Direction_BkPlcMc;
    Velocity:               LREAL;
    Acceleration:           LREAL;
    Deceleration:           LREAL;
    Jerk:                   LREAL;
    SetPosition:            LREAL;
    DetectionVelocityLimit: LREAL;
    DetectionVelocityTime:  LREAL;
    TorqueLimit:            LREAL;
    TorqueTolerance:        LREAL;
    TimeLimit:              TIME;
    DistanceLimit:          LREAL;
    BufferMode:             MC_BufferMode_BkPlcMc:=Aborting_BkPlcMc;
END_VAR

312

Version: 1.8.3

TF5810

MC_StepBlock_BkPlcMcExecute  BOOLDirection  MC_Direction_BkPlcMcVelocity  LREALAcceleration  LREALDeceleration  LREALJerk  LREALSetPosition  LREALDetectionVelocityLimit  LREALDetectionVelocityTime  TIMETorqueLimit  LREALTorqueTolerance  LREALTimeLimit  TIMEDistanceLimit  LREALBufferMode  MC_BufferMode_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  DoneBOOL  BusyBOOL  ActiveBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

Name
Execute
Direction

Velocity

Acceleration

Type
BOOL
MC_Direction_Bk
PlcMc

LREAL

LREAL

Deceleration

LREAL

Jerk

SetPosition
DetectionVelocityLimit
DetectionVelocityTime

TorqueLimit
TorqueTolerance
TimeLimit

DistanceLimit

BufferMode

LREAL

LREAL
LREAL
LREAL

LREAL
LREAL
TIME

LREAL

MC_BufferMode_
BkPlcMc

Description
The homing is initiated by a rising edge at this input.

The direction is specified via MC_Direction_BkPlcMc
[} 124].
[mm/s] The required motion velocity in actual value units
of the axis per second.
[mm/s2] The required acceleration in actual value units of
the axis per square second. If this parameter is 0.0, it is
replaced by a default value from the axis parameters.
[mm/s2] The required deceleration in actual value units of
the axis per square second. If this parameter is 0.0, it is
replaced by a default value from the axis parameters.
[mm/s3] The required jerk in actual value units of the axis
per square second. If this parameter is 0.0, it is replaced
by a default value from the axis parameters.
Required position on the referencing cam.
[mm/s] Velocity limit to reliably detect a stop
[s] Time in which the actual velocity must be below the
DetectionVelocityLimit in order to reliably detect the fixed
stop.
[Bar] Limitation for the pressure
[Bar] Tolerance for the pressure
After this time, the referencing is canceled with error. At
zero, time monitoring is disabled.
After this distance, the referencing will be aborted with an
error. At zero, the distance monitoring is disabled.
reserved

 Inputs/outputs

VAR_IN_OUT
    Axis:               AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Done:               BOOL;
    Busy:               BOOL;
    Active:             BOOL;
    CommandAborted:     BOOL;
    Error:              BOOL;
    ErrorID:            UDINT;
END_VAR

Name
Done
Busy
Active
CommandAborted
Error
ErrorID

Type
BOOL
BOOL
BOOL
BOOL
BOOL
UDINT

Description
Successful processing is indicated here
Indicates that a command is being processed.
Readiness for operation is indicated here.
Here an abort of the command is indicated.
The occurrence of an error is indicated here.
An encoded error message is provided here.

TF5810

Version: 1.8.3

313

PLCopen Motion Control

Behavior of the function block

On a rising edge at Execute all parameters are transferred to the function block
MC_StepBlockDetection_BkPlcMc [} 314]. If the internal function block MC_StepBlockDetection_BkPlcMc is
successfully processed, the determined position is set accordingly via MC_SetPosition_BkPlcMc [} 43].

During processing, the function block reports Busy and Active. After successful position setting, Done is
reported back. If a subordinate function block reports an error, this is reported via Error and ErrorID.

4.6.2.4

MC_StepBlockDetection_BkPlcMc

The function block is used for referencing via a limit switch. The function block outputs the position of the
cam to the outside via RecordedPosition. No position is set.

 Inputs
VAR_INPUT
    Execute:                 BOOL;
    Direction:               MC_Direction_BkPlcMc;
    Velocity:                LREAL;
    Acceleration:            LREAL;
    Deceleration:            LREAL;
    Jerk:                    LREAL;
    SetPosition:             LREAL;
    DetectionVelocityLimit:  LREAL;
    DetectionVelocityTime:   LREAL;
    TorqueLimit:             LREAL;
    TorqueTolerance:         LREAL;
    TimeLimit:               TIME;
    DistanceLimit:           LREAL;
    BufferMode:              MC_BufferMode_BkPlcMc:=Aborting_BkPlcMc;
END_VAR

314

Version: 1.8.3

TF5810

MC_StepBlockDetection_BkPlcMcExecute  BOOLDirection  MC_Direction_BkPlcMcVelocity  LREALAcceleration  LREALDeceleration  LREALJerk  LREALDetectionVelocityLimit  LREALDetectionVelocityTime  TIMETorqueLimit  LREALTorqueTolerance  LREALTimeLimit  TIMEDistanceLimit  LREALBufferMode  MC_BufferMode_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  DoneBOOL  BusyBOOL  ActiveBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDLREAL  RecordedPositionPLCopen Motion Control

Description
The homing is initiated by a rising edge at this input.
A direction preset coded according to
MC_Direction_BkPlcMc [} 124].
[mm/s] The required motion velocity in actual value units
of the axis per second.
[mm/s2] The required acceleration in actual value units of
the axis per square second. If this parameter is 0.0, it is
replaced by a default value from the axis parameters.
[mm/s2] The required deceleration in actual value units of
the axis per square second. If this parameter is 0.0, it is
replaced by a default value from the axis parameters.
[mm/s3] The required jerk in actual value units of the axis
per square second. If this parameter is 0.0, it is replaced
by a default value from the axis parameters.
Required position on the referencing cam.
[mm/s] Velocity limit to reliably detect a stop
[s] Time in which the actual velocity must be below the
DetectionVelocityLimit in order to reliably detect the fixed
stop.
[Bar] Limitation for the pressure
[Bar] Tolerance for the pressure
After this time, the referencing is canceled with error. At
zero, time monitoring is disabled.
After this distance, the referencing will be aborted with an
error. At zero, the distance monitoring is disabled.

Name
Execute
Direction

Velocity

Acceleration

Type
BOOL
MC_Direction_BkPl
cMc

LREAL

LREAL

Deceleration

LREAL

Jerk

SetPosition
DetectionVelocityLimit
DetectionVelocityTime

TorqueLimit
TorqueTolerance
TimeLimit

DistanceLimit

BufferMode

LREAL

LREAL
LREAL
LREAL

LREAL
LREAL
TIME

LREAL

MC_BufferMode_B
kPlcMc

 Inputs/outputs

VAR_IN_OUT
    Axis:               AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Done:               BOOL;
    Busy:               BOOL;
    Active:             BOOL;
    CommandAborted:     BOOL;
    Error:              BOOL;
    ErrorID:            UDINT;
    RecordedPosition:   LREAL;
END_VAR

TF5810

Version: 1.8.3

315

PLCopen Motion Control

Name
Done
Busy
Active
CommandAborted
Error
ErrorID
RecordedPosition

Type
BOOL
BOOL
BOOL
BOOL
BOOL
UDINT
LREAL

Description
Successful processing is indicated here
Indicates that a command is being processed.
Readiness for operation is indicated here.
Here an abort of the command is indicated.
The occurrence of an error is indicated here.
An encoded error message is provided here.
[mm] Determined position of the referencing cam.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. A number of problems
can be detected and reported during this process:

• At Direction, MC_Positive_Direction_BkPlcMc, MC_Negative_Direction_BkPlcMc, is accepted

• The DistanceLimit must have a value greater than the increment resolution.

The parameters Velocity, Acceleration, Deceleration and Jerk are transferred to
MC_MoveVelocity_BkPlcMc [} 79]. After the cam has been detected, the position is communicated via
RecordedPosition and a MC_Halt_BkPlcMc [} 71] stops the movement.

A hard stop is detected when the actual velocity for the time DetectionVelocityTime is below
DetectionVelocityLimit or the current pressure is greater than TorqueLimit minus TorqueTolerance. If no
fixed stop is detected within DistanceLimit or TimeLimit, referencing is aborted with an error.

During processing, the function block reports Busy and Active. After successful processing Done is
reported back. If a subordinate function block reports an error, this is reported via Error and ErrorID.

4.6.2.5

MC_StepReferencePulse_BkPlcMc

The function block is used to reference an incremental position measuring system. An actual value setting is
carried out internally via the referencing pulse.

 Inputs
VAR_INPUT
    Execute:              BOOL;
    Direction:            MC_Direction_BkPlcMc;
    Velocity:             LREAL;
    Acceleration:         LREAL;
    Deceleration:         LREAL;
    Jerk:                 LREAL;
    SetPosition:          LREAL;
    TorqueLimit:          LREAL;
    TimeLimit:            TIME;
    DistanceLimit:        LREAL;
    BufferMode:           MC_BufferMode_BkPlcMc:=Aborting_BkPlcMc;
END_VAR

316

Version: 1.8.3

TF5810

MC_StepReferencePulse_BkPlcMcExecute  BOOLDirection  MC_Direction_BkPlcMcVelocity  LREALAcceleration  LREALDeceleration  LREALJerk  LREALSetPosition  LREALTorqueLimit  LREALTimeLimit  TIMEDistanceLimit  LREALBufferMode  MC_BufferMode_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  DoneBOOL  BusyBOOL  ActiveBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDPLCopen Motion Control

Description
The homing is initiated by a rising edge at this input.
The direction is specified via MC_Direction_BkPlcMc.
[mm/s] The required motion velocity in actual value
units of the axis per second.
[mm/s2] The required acceleration in actual value units
of the axis per square second. If this parameter is 0.0, it
is replaced by a default value from the axis parameters.
[mm/s2] The required deceleration in actual value units
of the axis per square second. If this parameter is 0.0, it
is replaced by a default value from the axis parameters.
[mm/s3] The required jerk in actual value units of the
axis per square second. If this parameter is 0.0, it is
replaced by a default value from the axis parameters.
Required position on the referencing cam.
Reserved
After this time, the referencing is canceled with error. At
zero, time monitoring is disabled.
After this distance, the referencing will be aborted with
an error. At zero, the distance monitoring is disabled.
Reserved

Name
Execute
Direction
Velocity

Type
BOOL
MC_Direction_BkPlcMc
LREAL

Acceleration

LREAL

Deceleration

LREAL

Jerk

LREAL

SetPosition
TorqueLimit
TimeLimit

LREAL
LREAL
TIME

DistanceLimit

LREAL

BufferMode

MC_BufferMode_BkPlcMc

 Inputs/outputs

VAR_IN_OUT
    Axis:               AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Done:               BOOL;
    Busy:               BOOL;
    Active:             BOOL;
    CommandAborted:     BOOL;
    Error:              BOOL;
    ErrorID:            UDINT;
END_VAR

Name
Done
Busy
Active
CommandAborted
Error
ErrorID

Type
BOOL
BOOL
BOOL
BOOL
BOOL
UDINT

Description
Successful processing is indicated here
Indicates that a command is being processed.
Readiness for operation is indicated here.
Here an abort of the command is indicated.
The occurrence of an error is indicated here.
An encoded error message is provided here.

Behavior of the function block

On a rising edge at Execute the parameters Direction, Velocity, Acceleration and Deceleration are
transferred to the MC_StepReferencePulseDetection_BkPlcMc [} 318] function block. If the internal function
block MC_StepReferencePulseDetection_BkPlcMc is processed successfully, the determined position is set
accordingly via MC_SetPosition_BkPlcMc [} 43].

TF5810

Version: 1.8.3

317

PLCopen Motion Control

During processing, the function block reports Busy and Active. After successful position setting, Done is
reported back. If a subordinate function block reports an error, this is reported via Error and ErrorID.

4.6.2.6

MC_StepReferencePulseDetection_BkPlcMc

The function block is used to reference an incremental position measuring system. The function block
outputs the position of the reference pulse to the outside via RecordedPosition. No position is set.

 Inputs
VAR_INPUT
    Execute:              BOOL;
    Direction:            MC_Direction_BkPlcMc;
    Velocity:             LREAL;
    Acceleration:         LREAL;
    Deceleration:         LREAL;
    Jerk:                 LREAL;
    SetPosition:          LREAL;
    TorqueLimit:          LREAL;
    TimeLimit:            TIME;
    DistanceLimit:        LREAL;
    BufferMode:           MC_BufferMode_BkPlcMc:=Aborting_BkPlcMc;
END_VAR

Name
Execute
Direction

Type
BOOL
MC_Direction_BkPlcMc

Velocity

LREAL

Acceleration

LREAL

Deceleration

LREAL

Jerk

LREAL

SetPosition
TorqueLimit
TimeLimit

LREAL
LREAL
TIME

DistanceLimit

LREAL

BufferMode

MC_BufferMode_BkPlcMc

Description
The homing is initiated by a rising edge at this input.

The direction is specified via MC_Direction_BkPlcMc
[} 124].
[mm/s] The required motion velocity in actual value units
of the axis per second.
[mm/s2] The required acceleration in actual value units of
the axis per square second. If this parameter is 0.0, it is
replaced by a default value from the axis parameters.
[mm/s2] The required deceleration in actual value units of
the axis per square second. If this parameter is 0.0, it is
replaced by a default value from the axis parameters.
[mm/s3] The required jerk in actual value units of the axis
per square second. If this parameter is 0.0, it is replaced
by a default value from the axis parameters.
Required position on the referencing cam.
Reserved
After this time, the referencing is canceled with error. At
zero, time monitoring is disabled.
After this distance, the referencing will be aborted with an
error. At zero, the distance monitoring is disabled.
Reserved

318

Version: 1.8.3

TF5810

MC_StepReferencePulseDetection_BkPlcMcExecute  BOOLDirection  MC_Direction_BkPlcMcVelocity  LREALAcceleration  LREALDeceleration  LREALJerk  LREALTorqueLimit  LREALTimeLimit  TIMEDistanceLimit  LREALBufferMode  MC_BufferMode_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  DoneBOOL  BusyBOOL  ActiveBOOL  CommandAbortedBOOL  ErrorUDINT  ErrorIDLREAL  RecordedPositionPLCopen Motion Control

 Inputs/outputs

VAR_IN_OUT
    Axis:               AXIS_REF_BkPlcMc;
END_VAR

Name
Axis

Type
AXIS_REF_BkPlcMc

Description

Here, the address of a variable of type AXIS_REF_BkPlcMc [} 86] should
be transferred.

 Outputs

VAR_OUTPUT
    Done:               BOOL;
    Busy:               BOOL;
    Active:             BOOL;
    CommandAborted:     BOOL;
    Error:              BOOL;
    ErrorID:            UDINT;
    RecordedPosition:   LREAL;
END_VAR

Name
Done
Busy
Active
CommandAborted
Error
ErrorID
RecordedPosition

Type
BOOL
BOOL
BOOL
BOOL
BOOL
UDINT
LREAL

Description
Successful processing is indicated here
Indicates that a command is being processed.
Readiness for operation is indicated here.
Here an abort of the command is indicated.
The occurrence of an error is indicated here.
An encoded error message is provided here.
[mm] Determined position at the reference pulse.

Behavior of the function block

On a rising edge at Execute the function block checks the transferred axis interface. During this process,
problems may be detected and reported:

• At Direction, MC_Positive_Direction_BkPlcMc, MC_Negative_Direction_BkPlcMc, is accepted

The parameters Velocity, Acceleration, Deceleration and Jerk are transferred to
MC_MoveVelocity_BkPlcMc [} 79]. After the cam has been detected, the position is communicated via
RecordedPosition and a MC_Halt_BkPlcMc [} 71] stops the movement. If no reference pulse is detected
within DistanceLimit or TimeLimit the referencing is aborted with an error

During processing, the function block reports Busy and Active. After successful position setting, Done is
reported back. If a subordinate function block reports an error, this is reported via Error and ErrorID.

TF5810

Version: 1.8.3

319

Knowledge Base

5 Knowledge Base

Knowledge Base of the TcPlcHydraulics PLC library

Available from version 3.0

Here you will find a number of answers to recurring questions.

Topics

Name

Global Constants [} 338]

Commissioning [} 380]

SampleList [} 374]

Ideas Bank [} 321]

HMI tool [} 370]

Description
Pre-defined error codes, masks for bit queries, ADS
codes etc.
Commissioning information

Program examples

Tips and tricks

The PlcMcManager

Problems during library updates

Compilation problems may occur when the library is updated. The reason may be a change of name of one
or several function blocks or data types. Such changes are not always avoidable and generally implemented
for one of the following reasons:

• Adaptation to the rules of the PLC Open Motion Control definitions.

• Further development of the PLC Open Motion Control definitions.

• Further development the technology provided.

• Adaptation to the technology used, particularly support of further I/O devices.

• Avoidance of name collisions and other compatibility problems with other libraries.

From V3.0 build 22, the library uses TcEtherCAT.LIB for communication via the EtherCAT fieldbus. In older
TwinCAT environments this library is not yet available. If the TcPlcHydraulics library is to be used in such an
environment, the TcEtherCatDummy.LIB provided should be copied into the project directory and renamed
to TcEtherCAT.LIB. This library should then be added to the project BEFORE TcPlcHydraulics.LIB.

This procedure must not be used in TwinCAT environments that support EtherCAT. The file
provided must NOT be used to replace an existing operational TcEtherCAT.LIB.

There are no functions that require EtherCAT technologies.

The library version used in a project should be copied into the project directory and backed up
together with the project. This avoids inadvertent version changes, which could otherwise occur if
TwinCAT is updated in the meantime. To update the library, copy the new version directly into the
project directory.

We strongly recommend carrying out a trial compilation of the whole project after a library update. In
addition, the mapping should be updated with the System Manager. If the table shown below
indicates a change in size in one of the structures, it is essential to check the address assignment.

If the library is updated to a version that differs not only in the third (build) number, but also in the
major and minor version number, it can be assumed that the mappings created by the System
Manager are no longer correct. In this case it is imperative to refresh the links.

320

Version: 1.8.3

TF5810

Old name
ST_TcMcAxInterface

New name
Axis_Ref_BkPlcMc

ST_TcPlcMcCamId

MC_CAM_ID_BkPlcMc

ST_TcPlcMcCamRef

MC_CAM_REF_BkPlcMc

E_TcMCDirection

MC_Direction_BkPlcMc

E_TcMCStartMode

MC_StartMode_BkPlcMc

ST_TcPlcMcEncoderIn

ST_TcPlcMcEncoderOut

ST_TcPlcMcDriveIn

ST_TcPlcMcDriveOut

ST_TcPlcMcAx2000In

ST_TcPlcMcAx2000Out

MC_AxUtiCancelMonitoring_BkPlc
Mc

Size of the I/O structures in bytes

---

---

---

---

---

---

---

Knowledge Base

Reason of for the change
Adaptation to PLC Open Motion
Control definitions.
Adaptation to PLC Open Motion
Control definitions.
Adaptation to PLC Open Motion
Control definitions.
Adaptation to PLC Open Motion
Control definitions.
Adaptation to PLC Open Motion
Control definitions.
Omitted; task is handled by
ST_TcPlcDeviceInput
Omitted; task is handled by
ST_TcPlcDeviceOutput
Omitted; task is handled by
ST_TcPlcDeviceInput
Omitted; task is handled by
ST_TcPlcDeviceOutput
Omitted; task is handled by
ST_TcPlcDeviceInput
Omitted; task is handled by
ST_TcPlcDeviceOutput
Omitted; redundant due to PLC
Open definitions

from V3.0.0
-
-
-
-
-
-
143

from V3.1.0 (proposed)
-
-
-
-
-
-
?

103

?

V 2.1.X
16

Name
ST_TcPlcMcEncoderIn
ST_TcPlcMcEncoderOut 1
ST_TcPlcMcDriveIn
ST_TcPlcMcDriveOut
ST_TcPlcMcAx2000In
ST_TcPlcMcAx2000Out

ST_TcPlcDeviceInput
[} 149]

23
40
37
26
-

ST_TcPlcDeviceOutput
[} 153]

-

5.1

FAQs

Available from version 3.0

Here you will find answers to frequently asked questions.

TF5810

Version: 1.8.3

321

Knowledge Base

Description

Name
FAQ #1 [} 322] How do I integrate one or more axes into a PLC application?
FAQ #2 [} 323] What data has to be created in the PLC application for the axes?
FAQ #3 [} 323] How do I initialize the data and load the parameters for an axis when the PLC starts?
FAQ #4 [} 324] How is the actual position of the axes determined?
FAQ #5 [} 327] How is the control value for an axis created?
FAQ #6 [} 327] How is the control value for an axis prepared for output?
FAQ #7 [} 327] How is the control value output to an axis?

In what order should the function blocks of an axis be called?

FAQ #8 [} 329]
FAQ #9 [} 329] How do I control a valve output stage (on-board or externally)?
FAQ #10 [} 329] How do I create a message buffer?
FAQ #11 [} 330] How do I abort monitoring of a function?
FAQ #12 [} 331] How do I monitor the communication with an I/O device?
FAQ #13 [} 331] How do I assign my own labels to customer-specific axis parameters?
FAQ #14 [} 331] How do I control a current valve?
FAQ #15 [} 331] Which axis variables should be logged with the Scope?
FAQ #16 [} 332] What is the purpose of the variable nDebugTag in Axis_Ref_BkPlcMc?
FAQ #17 [} 332] What has to be taken into account when Sercos drives are used?
FAQ #18 [} 333] How is a pressure or a force determined?
FAQ #19 [} 333] What has to be taken into account when AX5000 drives are used?
FAQ #20 [} 334] How do I prepare an axis for blending based on PLC Open?
FAQ #21 [} 336] How can I access registers of a terminal, to which an encoder or a valve of an axis is

connected?

FAQ #22 [} 336] What is the structure of an ASCII file for a linearization table?
FAQ #23 [} 337] How can PlcMcManager commands be blocked?
FAQ #24 [} 338] What format do import/export files with characteristic curve data have?

Commissioning
[} 380]

How is operation of the axis begun, and how is it optimized?

FAQ #1 How do I integrate one or more axes into a PLC application?

The procedure here differs fundamentally from an axis guided by the NC task, because in this case
everything done by the NC task is performed by the PLC. Ready-made function blocks are, however,
available in most areas, so that the additional programming effort is held within reasonable limits. The
following particular points must be considered:

• Axis data in the PLC application (FAQ #2 [} 323])

• Initializing and loading the axis parameters when starting the PLC application (FAQ #3 [} 323])

• Acquisition of actual values (FAQ #4 [} 324])

• Generating control values (FAQ #5 [} 327])

• Processing control values in preparation for output (FAQ #6 [} 327])

• Setting up the axes (Commissioning [} 380])

• Commissioning of actual pressure determination with function blocks of type

MC_AxRtReadPressureSingle_BkPlcMc [} 222] or MC_AxRtReadPressureDiff_BkPlcMc [} 220].

• Organization of the procedure for movement (FAQ #7 [} 327])

322

Version: 1.8.3

TF5810

Knowledge Base

If only the usual blocks (encoder, generator, finish, drive) for the axis are to be called, a block of
type MC_AxStandardBody_BkPlcMc should be used for simplicity.

FAQ #2 What data has to be created in the PLC application for the axes?

For each axis, one variables of each type Axis_Ref_BkPlcMc [} 86], ST_TcPlcDeviceInput [} 149] and
ST_TcPlcDeviceOutput [} 153] has to be created. The use of variable fields is highly recommended for
multiple axes. Examples for one and five axes can be found in the first sample programs.

The procedure using MC_AxUtiStandardInit_BkPlcMc [} 254] function blocks shown in these examples
ensures correct initialization on start-up of the PLC and initiates loading of the axis parameters from files.

Further data are required for realizing message logging. See also FAQ #10 [} 329].

Further data are required for assigning one's own IDs to customer-specific axis parameters in the
PlcMcManager. See also FAQ #13 [} 331]

Further data are required in order to utilize blending according to PLC Open. See also FAQ #20 [} 334].

FAQ #3 How do I initialize the data for an axis?

A number of initializations must be carried out when the PLC applications starts. This is best done in three
stages, which are provided by an MC_AxUtiStandardInit_BkPlcMc [} 254] function block and should only be
realized directly by the application in special cases. They are described here only for the sake of
completeness.

1. A number of pointers must be correctly set up to link the components of the axes together. This task

should be solved with a function block of type MC_AxUtiStandardInit_BkPlcMc [} 254], which detects a
shift or change in size in the memory or the change of a type code during a subsequent online change
and then ensures that the pointers are reinitialised and the parameters are reloaded.

2. The parameters for the axis must be appropriately set. Although it would be technically possible for the
application to do have these assignments hard-coded, this is not usually helpful. It is preferable to
save the settings in files, which are loaded on system startup under control of the application through
the MC_AxUtiStandardInit_BkPlcMc [} 254] function block. Notes on setting up an axis can be found
under Commissioning [} 380].

3. The task cycle time should be applied in the axis parameters. This should be done at the end of the

parameter loading procedure, in order to set this value correctly, in view of the fact that it is important
for the function of many function blocks. An MC_AxUtiStandardInit_BkPlcMc [} 254] function block
deals with this task automatically.

If a function block of type MC_AxAdsCommServer_BkPlcMc is used in the application, the function
block must be called in the same task that carries out the pointer assignments. If this is not
possible, or for some reason difficult, then calling the function block must be prevented while the
assignments are being carried out. The result, otherwise, can be that the PLC application crashes
as a result of serious runtime errors (Page Fault Exception).

All activities listed here should through be realized and coordinated by an
MC_AxUtiStandardInit_BkPlcMc function block. If the nInitState variable in Axis_Ref_BkPlcMc of
the axis adopts either the value 2 or -2, then the initialization has been successful or has ended with
an error. If the initialization is successful, MC_AxUtiStandardInit_BkPlcMc.Ready and
bParamsEnable in Axis_Ref_BkPlcMc are TRUE, otherwise this variable remains FALSE.

The sample programs provided specify the name of the axis and the name (included the path) of
the corresponding parameter file. It is essential that these specifications are modified to match the
particular application.

TF5810

Version: 1.8.3

323

Knowledge Base

FAQ #4: How is the actual position of the axes determined?

A range of signal transducers may be considered for use as position sensors, operating according to a
variety of physical principles to generate a position-dependent electrical magnitude. This magnitude
determines the type of I/O components that must be used. The variables of types ST_TcPlcDeviceInput
[} 149] and ST_TcPlcDeviceOutput [} 153] must be created for each axis, and contain elements that are to be
linked to the actual value, counter, latch, control and status variables associated with the I/O hardware.

Here are a few examples:

324

Version: 1.8.3

TF5810

Knowledge Base

Encoder Type

iTcMc_EncoderAx2000_B110A
[} 199]

iTcMc_EncoderAx2000_B110R
[} 188]

iTcMc_EncoderAx2000_B200R
[} 189]

iTcMc_EncoderAx2000_B750A
[} 202]

iTcMc_EncoderAx2000_B900R
[} 189]

iTcMc_EncoderAX5000_B110A
[} 202]

iTcMc_EncoderCoE_DS402A [} 204]

iTcMc_EncoderCoE_DS402SR
[} 205]

iTcMc_EncoderEL3102 [} 208]

iTcMc_EncoderEL3142 [} 208]

iTcMc_EncoderEL3162 [} 208]

iTcMc_EncoderEL3162 [} 208]

iTcMc_EncoderEL5001 [} 208]

EtherCAT

EtherCAT

EtherCAT

-10V .. 10V

0mA .. 20mA

0 .. 10V

Potentiometric displacement
transducer
SSI

I/O component
AX2000 B110 with absolute
encoder

Signal
EtherCAT

AX2000 B110 with resolver

EtherCAT

AX2000 B200 with resolver

EtherCAT

AX2000 B750 with absolute
encoder

EtherCAT

AX2000 B900 with resolver

EtherCAT

AX5000 B110 with multi-turn
absolute encoder

EtherCAT servo controllers with
CoE DS402 support and
multi-turn encoder
EtherCAT servo controllers with
CoE DS402 support and resolver
or single-turn encoder
EL3102

EL3142

EL3162

EL3255

EL5001

EL5101

EL7041

EtherCAT encoder with
CoE_DS406 profile
IE5009

IP5009

KL10xx

KL11xx

KL12xx

KL13xx

KL14xx

KL17xx

KL10xx

KL11xx

KL12xx

KL13xx

KL14xx

KL17xx

TF5810

A/B increments, RS422="TTL"

iTcMc_EncoderEL5101 [} 210]

A/B increments, RS422="TTL"

iTcMc_EncoderEL7041 [} 210]

EtherCAT

SSI

SSI

2 bit, A/B increments

2 bit, A/B increments

2 bit, A/B increments

2 bit, A/B increments

2 bit, A/B increments

2 bit, A/B increments

4 bit, position cams

4 bit, position cams

4 bit, position cams

4 bit, position cams

4 bit, position cams

4 bit, position cams

iTcMc_EncoderCoE_DS406 [} 206]

iTcMc_EncoderIx5009 [} 211]

iTcMc_EncoderIx5009 [} 211]

iTcMc_EncoderDigIncrement
[} 208]

iTcMc_EncoderDigIncrement
[} 208]

iTcMc_EncoderDigIncrement
[} 208]

iTcMc_EncoderDigIncrement
[} 208]

iTcMc_EncoderDigIncrement
[} 208]

iTcMc_EncoderDigIncrement
[} 208]

iTcMc_EncoderDigCam [} 207]

iTcMc_EncoderDigCam [} 207]

iTcMc_EncoderDigCam [} 207]

iTcMc_EncoderDigCam [} 207]

iTcMc_EncoderDigCam [} 207]

iTcMc_EncoderDigCam [} 207]

Version: 1.8.3

325

Knowledge Base

I/O component
KL2521

KL2531

KL2541

KL2542

KL3001

KL3002

KL3011

KL3012

KL3021

KL3022

KL3041

KL3042

KL3044

KL3051

KL3052

KL3054

KL3061

KL3062

KL3064

KL3162

KL5001

KL5101

KL5111

M2510

M3100

M3120

Signal
Pulse Train

Stepper motor, direct (encoder
emulated through pulse counter)
Stepper motor, direct (with encoder
or encoder emulates through pulse
counter)
DC motor, direct with encoder

-10V .. 10V

-10V .. 10V

0mA .. 20mA

0mA .. 20mA

4mA .. 20mA

4mA .. 20mA

0mA .. 20mA

0mA .. 20mA

0mA .. 20mA

4mA .. 20mA

4mA .. 20mA

4mA .. 20mA

0V .. 10V

0V .. 10V

0V .. 10V

0V .. 10V

SSI

Encoder Type

iTcMc_EncoderKL2521 [} 211]

iTcMc_EncoderKL2531 [} 211]

iTcMc_EncoderKL2541 [} 212]

iTcMc_EncoderKL2542 [} 212]

iTcMc_EncoderKL3002 [} 213]

iTcMc_EncoderKL3002 [} 213]

iTcMc_EncoderKL3042 [} 213]

iTcMc_EncoderKL3042 [} 213]

iTcMc_EncoderKL3042 [} 213]

iTcMc_EncoderKL3042 [} 213]

iTcMc_EncoderKL3042 [} 213]

iTcMc_EncoderKL3042 [} 213]

iTcMc_EncoderKL3042 [} 213]

iTcMc_EncoderKL3042 [} 213]

iTcMc_EncoderKL3042 [} 213]

iTcMc_EncoderKL3042 [} 213]

iTcMc_EncoderKL3062 [} 213]

iTcMc_EncoderKL3062 [} 213]

iTcMc_EncoderKL3062 [} 213]

iTcMc_EncoderKL3162 [} 213]

iTcMc_EncoderKL5001 [} 213]

A/B increments, RS422="TTL"

iTcMc_EncoderKL5101 [} 214]

A/B increments, RS422="HTL"

iTcMc_EncoderKL5111 [} 214]

-10V .. 10V

iTcMc_EncoderM2510 [} 215]

A/B increments, RS422="TTL"

iTcMc_EncoderM3120 [} 215]

A/B increments, RS422="TTL"

iTcMc_EncoderM3120 [} 215]

If one of the components mentioned here is used, then one of the encoder function blocks provided will
usually be applied. The interfaces of these function blocks are not guaranteed and should therefore not be
called directly by the application. It is better to set the encoder type according to the constants in
E_TcMcEncoderType [} 98] under nEnc_Type in ST_TcHydAxParam [} 130], and to use a function block of
type MC_AxRtEncoder_BkPlcMc [} 198]. This then automatically calls the correct type of sub-function-block
for the type concerned.

All encoder function blocks use the parameters fEnc_IncWeighting and fEnc_IncInterpolation as increment
assessment. fEnc_ZeroShift is also used as a zero shift for absolute displacement sensors. Incremental
sensors usually require a reference travel using a MC_Home_BkPlcMc [} 68] function block, during which
fEnc_RefShift in ST_TcHydAxRtData [} 141] is determined. This value then does the job of the zero shift. It
goes without saying that in special cases the zero shift can also be defined with an MC_SetPosition_BkPlcMc
[} 43] function block. The referenced status of the axis should be defined with MC_SetReferenceFlag_BkPlcMc
[} 45]().

If it is not possible to determine the actual position with function blocks from the library for technical reasons,
this task can be handled by application function blocks, and the result can be entered in fActPos, and
fActVelo can be entered in ST_TcHydAxRtData [} 141], if required. For the sake of uniformity use should
again be made here of the fEnc_IncWeighting, fEnc_IncInterpolation and fEnc_ZeroShift or fEnc_RefShift
parameters.

326

Version: 1.8.3

TF5810

Knowledge Base

If only the usual function blocks (encoder, generator, finish, drive) for the axis are to be called, a
function block of type MC_AxStandardBody_BkPlcMc should be used for simplicity.

Commissioning of an actual pressure determination with function blocks of type
MC_AxRtReadPressureSingle_BkPlcMc or MC_AxRtReadPressureDiff_BkPlcMc is described in
the documentation for the function block.

FAQ #5: How is the control value for an axis created?

In each cycle, the PLC application must call a function block of type MC_AxRuntime_BkPlcMc [} 237], or
alternatively a suitable controller function block (e.g. a pressure regulator) for each axis. The parameter
nProfileType in ST_TcHydAxParam [} 130] specifies the procedure that is to be used to generate the control
value. Velocity control values are calculated here according to the type, and depending on other parameters
associated with the axis and on the movement data. These control values are, however, normalized to the
abstract numerical range ±1.0, and have not yet been prepared for immediate output to I/O hardware.

If only the usual function blocks (encoder, generator, finish, drive) for the axis are to be called, a
function block of type MC_AxStandardBody_BkPlcMc [} 253]should be used for simplicity.

FAQ #6: How is the control value for an axis prepared for output?

After calling the MC_AxRuntime_BkPlcMc [} 237] function block, a function block of type
MC_AxRtFinish_BkPlcMc [} 246] must be called for each axis. This function block assembles a number of
velocity components (control value, controller output, offset compensation, overlap compensation), and also
takes into account in the bends in the feed forward characteristic curve.

Numerical adjustment is usually necessary prior to output to an I/O module. An MC_AxRtDrive_BkPlcMc
[} 187] function block is to be called for each axis for this purpose. The value of nDrive_Type in
ST_TcHydAxParam [} 130] selects the hardware-specific sub-function-block to be used.

The variables of types ST_TcPlcDeviceInput [} 149] and ST_TcPlcDeviceOutput [} 153] must be created for
each axis, and contain elements that are to be linked to the set value and control variables of the I/O
hardware.

If only the usual function blocks (encoder, generator, finish, drive) for the axis are to be called, a
function block of type MC_AxStandardBody_BkPlcMc [} 253] should be used for simplicity.

FAQ #7: How is the control value output to an axis?

A range of devices and equipment might be functioning as actuators, applying a variety of physical principles
to create a variable velocity that depends on an electrical magnitude. This magnitude determines the type of
I/O components that must be used. The variables of types ST_TcPlcDeviceInput [} 149] and
ST_TcPlcDeviceOutput [} 153] must be created for each axis, and contain elements that are to be linked to
the variables of the I/O hardware.

Here are a few examples:

TF5810

Version: 1.8.3

327

Knowledge Base

I/O component
AX2000 B110 with absolute
encoder
AX2000 B110 with resolver

AX2000 B200 with resolver

AX2000 B750 with absolute
encoder
AX2000 B900 with resolver

AX5000 B110 with absolute
encoder
EtherCAT servo controllers with
CoE DS402 support and resolver,
single-turn or multi-turn encoder
EtherCAT valve with CoE_DS408
profile
EL2535
EL4031, EL4032, EL4034, EL4038
EL4131, EL4132, EL4134
EL4011, EL4012, EL4014, EL4018,
EL4112
EL4021, EL4022, EL4024,
EL4028,
EL4122, EL4124
EL7031

EL7041

IE2512

IP2512

Signal
EtherCAT

EtherCAT

EtherCAT

EtherCAT

EtherCAT

EtherCAT

EtherCAT

EtherCAT

PWM
-10 V .. 10 V

0..20 mA

4..20 mA

Stepper motor, direct

Stepper motor, direct

PWM

PWM

KL20xx, KL21xx, KL22xx, KL24xx 5 bit for operating a frequency

converter with fixed frequencies

KL20xx, KL21xx, KL22xx, KL24xx 4 bit for operating a voltage-

controlled stepper motor

Pulse Train

Stepper motor, direct

Drive Type

iTcMc_DriveAX2000_B110A [} 188]

iTcMc_DriveAX2000_B110R [} 188]

iTcMc_DriveAX2000_B200R [} 189]

iTcMc_DriveAx2000_B750A [} 189]

iTcMc_DriveAX2000_B900R [} 189]

iTcMc_DriveAX5000_B110A [} 189]

iTcMc_DriveCoE_DS402 [} 189]

iTcMc_Drive_CoE_DS408 [} 190]

iTcMc_DriveEL2535

iTcMc_DriveEL4132 [} 191]

iTcMc_DriveEL4x22

iTcMc_DriveEL7031 [} 192]

iTcMc_DriveEL7041 [} 193]

iTcMc_DriveIx2512_1Coil [} 190]

iTcMc_DriveIx2512_2Coil [} 190]

iTcMc_DriveIx2512_1Coil [} 190]

iTcMc_DriveIx2512_2Coil [} 190]

iTcMc_DriveLowCostInverter [} 197]

iTcMc_DriveLowCostStepper
[} 197]

iTcMc_DriveKL2521 [} 194]

iTcMc_DriveKL2531 [} 194]

DC motor, direct with encoder

iTcMc_DriveKL2532 [} 195]

PWM

iTcMc_DriveKL2535_1Coil [} 195]

iTcMc_DriveKL2535_2Coil [} 195]

Stepper motor, direct

iTcMc_DriveKL2541 [} 195]

DC motor, direct with encoder

iTcMc_DriveKL2542 [} 196]

-10 V .. 10 V

-10 V .. 10 V

-10 V .. 10 V

-10 V .. 10 V

iTcMc_DriveKL4032 [} 196]

iTcMc_DriveKL4032 [} 196]

iTcMc_DriveKL4032 [} 196]

iTcMc_DriveM2400_D1 [} 198],
iTcMc_DriveM2400_D2,
iTcMc_DriveM2400_D3,
iTcMc_DriveM2400_D4

KL2521

KL2531

KL2532

KL2535

KL2541

KL2542

KL4031

KL4032

KL4034

M2400

328

Version: 1.8.3

TF5810

Knowledge Base

If one of the components mentioned here is used, then one of the drive function blocks provided will usually
be used. These interfaces of these function blocks are not guaranteed and should therefore not be called
directly by the application. It is better to set the drive type according to the constants in E_TcMcDriveType
[} 94] under nDrive_Type in ST_TcHydAxParam [} 130], and to use a function block of type
MC_AxRtDrive_BkPlcMc [} 187].

If only the usual function blocks (encoder, generator, finish, drive) for the axis are to be called, a
function block of type MC_AxStandardBody_BkPlcMc [} 253] should be used for simplicity.

FAQ #8: In what order should the function blocks of an axis be called?

1. Obligatory: all function blocks, which detect the actual status of the axis. These include function blocks

of types MC_AxRtEncoder_BkPlcMc [} 198], MC_AxRtReadPressureDiff_BkPlcMc [} 220] or
MC_AxRtReadPressureSingle_BkPlcMc [} 222].

2. Usual: function blocks or commands, which update the enable signals of the axis. This is usually a
function block of type MC_Power_BkPlcMc [} 27]. For axes with an incremental encoder, which is
referenced using a cam, a function call MC_AxRtSetReferencingCamSignal_BkPlcMc is used in
addition.

3. Optional: Function blocks, which derive a decision or trigger a command based on an actual axis

status, an I/O signal or an application signal. For example, an axis start can be triggered in response
to the signal of a proximity limit switch, or an axis movement can be stopped before the target position
is reached, depending on the pressure increase.

4. Obligatory: Control value generators such as function blocks of type MC_AxRuntime_BkPlcMc [} 237].
5. Optional: Various controllers can be called at this point, as required. This can be a function block of

type MC_AxCtrlSlowDownOnPressure_BkPlcMc [} 178] or similar.

6. Obligatory: An adaptation function block of type MC_AxRtFinish_BkPlcMc [} 246].
7. Optional: If required, a function block for the automatic commissioning can be called at this point.

8. Obligatory: An output function block of type MC_AxRtDrive_BkPlcMc [} 187].

Instead of the library function blocks, application function blocks can be used. However, one should check
carefully whether this is necessary, in which case compatibility with the library must be ensured. In some
applications this may become necessary, in order to adapt a non-standard sensor or actuator, or to solve a
special control task.

FAQ #9: How do I control a valve output stage (on-board or externally)?

The ST_TcPlcDeviceOutput [} 153] structure is intended for the bPowerOn and bEnable signals and for
controlling the output stage supply and activation. Both signals are set by function blocks of type
MC_Power_BkPlcMc [} 27], if the input Enable is set. At the same time this function block sets the software
controller enable in ST_TcHydAxRtData [} 141].nDeCtrlDWord [} 339].

The ST_TcPlcDeviceInput [} 149] structure is intended for the signals bPowerOk, bEnAck and bReady for
the output stage supply control, feedback from the output stage activation and the status signal. The
differences in the signals provided by different manufacturer can be very significant. Currently, only the
bPowerOk signal is used for specifying the Status output of the MC_Power_BkPlcMc [} 27] function block. If
no suitable signal is available, or if no monitoring is to be realised, ST_TcHydAxParam
[} 130].bDrive_DefaultPowerOk should be set.

FAQ #10: How do I create a message buffer?

Direct output of messages from the function blocks would result in runtime variations that would be difficult to
calculate. For this reason, the messages are stored in a buffer and output in the Windows Event Viewer one
after another, if required.

In order to be able to use a message buffer, a variable of type ST_TcPlcMcLogBuffer [} 156] must be created.
This buffer is used to hold the messages from all axes. It is important that only one such variable is created
in the project, irrespective of the number of axes. The address of this buffer should be transferred to the
MC_AxUtiStandardInit_BkPlcMc [} 254] function blocks of all axes, together with the addresses of the other

TF5810

Version: 1.8.3

329

Knowledge Base

individual axis components. This function blocks are usually called in the initialization part of the project. This
address is stored in the element pStAxLogBuffer in the structure Axis_Ref_BkPlcMc [} 86] and by the function
block.

nLogLevel in Axis_Ref_BkPlcMc [} 86] is used to specify the significance level threshold for storing messages
in the buffer. The values [} 347] to be used are defined in the global variables of the library. Note that this
setting is required for each axis.

The library function blocks detect the preparations mentioned above and will commence issuing messages.
However, if the message output is enabled, the buffer would fill up quickly and not accept further messages.
There are two ways to avoid this.

FAQ #10.1: Passing on messages to the Windows Event Viewer

In order to transfer messages from the LogBuffer of the library to the Windows Event Viewer, a function
block of type MC_AxRtLoggerSpool_BkPlcMc [} 261] should be called cyclically. Witch each call a message is
removed from the LogBuffer.

Computers running Windows CE are also capable of amending an Event Viewer for the messages
created by TwinCAT. To this end this service is emulated by the TwinCAT system service.
However, usually only a flash disk will be available. In order to avoid overloading the relatively small
message capacity of the Event Viewer, only errors should be logged.

FAQ #10.2: Deleting the oldest messages

In order to ensure a minimum number of messages that can be handled, a function block of type
MC_AxRtLoggerDespool_BkPlcMc [} 259] should be called cyclically. With each call, this function block
removes the oldest message from the LogBuffer, until a transferred number of free messages is available.
The deleted messages are lost.

FAQ #10.3: Generating logger entries through the application

An application can output a message either axis-related or non-axis-related. The function blocks
MC_AxRtLogAxisEntry_BkPlcMc [} 256] and MC_AxRtLogEntry_BkPlcMc [} 258] are available for this purpose.

FAQ #10.4: Library-internal message buffer

A cyclic call to the MC_Communications_BkPlcMc [} 281] function block uses a library-internal message
buffer. For this purpose, the MC_Communications_BkPlcMc function block references the internal message
buffer in the transferred axes and calls the MC_AxRtLoggerSpool_BkPlcMc [} 261] function block. A message
buffer created in the PLC application is no longer required here. In addition, the message buffer referenced
when calling MC_AxUtiStandardInit_BkPlcMc [} 254] is ignored when calling the
MC_Communications_BkPlcMc function block.

FAQ #11: How do I abort monitoring of a function?

Some library function blocks start an activity, for which cyclic calling is no longer essential. However, these
function blocks are also structured according to the rules of the PLCopen Motion Control guidelines in such a
way that they fully monitor the activity and present it at their outputs. This is indicated by the output Busy,
which most function blocks provide.

Omitting the cyclic call of a function block that is in this monitoring state would usually result in significant
problems. The next function start with the respective function block would have problems with evaluating the
edges at its inputs, or it would detect that meanwhile the axis has executed another function and indicated a
problem that doesn't exist (CommandAborted).

In older versions of the library a function block of type MC_AxUtiCancelMonitoring_BkPlcMc() was provided,
which for a few motion functions aborted the monitoring by the function block initiating the function. This
function block is no longer required, in view of the fact that in the meantime the PLC Open rules have been
implemented more fully.

330

Version: 1.8.3

TF5810

Knowledge Base

To instruct a function block to terminate monitoring its function, in most cases it is sufficient to call it once or
several times with Execute:=FALSE. This applies in particular to MC_MoveAbsolute_BkPlcMc [} 73](),
MC_MoveRelative_BkPlcMc [} 77]() and MC_MoveVelocity_BkPlcMc [} 79]().

Subsequently, a new functionality can be started in same or a later cycle with the same function block or an
instance of the same or another type. This procedure can be repeated as required.

Complex functions composed of several sub-actions, such as MC_Home_BkPlcMc(), require the
continuous calling of the function block as the latter organizes the required processes itself
( MC_Home_BkPlcMc ()) [} 68]

FAQ #12: How do I monitor the communication with an I/O device?

ST_TcPlcDeviceInput [} 149] and ST_TcPlcDeviceOutput [} 153] variables provide an element with the name
uiBoxState. If the Bus Couplers or the interface cards of the power units used offer a corresponding variable
and the variable assumes the value 0 with undisturbed communication in the fieldbus used, a link should be
created. This is possible, for example, with Beckhoff Lightbus and Real-time Ethernet. If an
MC_Power_BkPlcMc [} 27] function block is used for the axis, the function block monitors the uiBoxState and
reports problems with the communication. In such a case the axis is put in an error state.

EtherCAT offers enhanced options.

FAQ #13: How do I assign my own labels to customer-specific axis parameters?

The Axis_Ref_BkPlcMc [} 86] structure uses the pAuxLabels pointer to support the application of an array of
texts, which are displayed by the PlcMcManager. These texts can be loaded by the
MC_AxUtiStandardInit_BkPlcMc [} 254] function block when the application is started from a file. To this end
this function block must be provided with the address of an ST_TcMcAuxDataLabels [} 149] variable and a
suitable file.

It goes without saying that it is also possible to define the elements of the ST_TcMcAuxDataLabels [} 149]
variable through direct assignment from the application. In this case, the file is not required.

A number of controller function blocks of the library define the arrays texts automatically.

FAQ #14: How do I control a current valve?

In contrast to a 4/2 or 3/2 directional proportional valve or a servo-valve, a current valve is controlled with a
0..10 V signal (if a valve output stage is present) or actuated with a load-independent current of 0...INominal. In
this control, only the magnitude of the velocity is transferred. The direction is transferred not with the sign,
but by other means. This usually requires digital signals, which are used for controlling switching valves. The
ST_TcPlcDeviceOutput [} 153] structure provides elements such as bBrakeOff, bMovePos and bMoveNeg
for this purpose. For generating an absolute control value, bDrive_AbsoluteOutput should be set in the axis
parameters.

This also enables the use of conventional frequency converters with asynchronous motor, encoder
and brake, if the converter provides an analog input.

FAQ #15: Which axis variables should be logged with the Scope?

The following signal composition is recommended:

• Always: actual axis position: Axis_Ref_BkPlcMc.ST_TcHydAxRtData [} 141].fActPos: in actual value

units, as specified by the encoder scaling.

• Only for gear or synchronization coupling, cam plate: set axis position:

Axis_Ref_BkPlcMc.ST_TcHydAxRtData.fSetPos: in actual value units, as specified by the encoder
scaling.

TF5810

Version: 1.8.3

331

Knowledge Base

• Particularly during commissioning: actual velocity value:

Axis_Ref_BkPlcMc.ST_TcHydAxRtData.fActVelo: velocity in physical representation.

• Particularly during commissioning: Residual distance or target position:

Axis_Ref_BkPlcMc.ST_TcHydAxRtData.fDistanceToTarget or
Axis_Ref_BkPlcMc.ST_TcHydAxRtData.fTargetPos: in actual value units, as specified by the encoder
scaling.

• Only if pressure/force logging is active: various actual pressure and force values: in

Axis_Ref_BkPlcMc.ST_TcHydAxRtData: as required fActPressure fActPressureA fActPressureB
fActForce fValvePressure fSupplyPressure: pressures and forces, unit is defined through
parameterization of the logging function blocks.

• Particularly during commissioning: velocity control value:

Axis_Ref_BkPlcMc.ST_TcHydAxRtData.fSetVelo: velocity in physical representation.

• Particularly during commissioning: controller output:

Axis_Ref_BkPlcMc.ST_TcHydAxRtData.fLagCtrlOutput: velocity in physical representation.

The signal selection in ScopeView is simplified if the Axis_Ref_BkPlcMc variables contain a name
that begins with aaa_. This approach is used in the sample programs and ensures that the variables
can be found quickly in the symbol list.

In the signal composition of ScopeView, channels can be temporarily disabled. In this way it is
possible to maintain a comprehensive configuration but limit logging to data that are currently of
interest.

FAQ #16: What is the purpose of the variable nDebugTag in Axis_Ref_BkPlcMc?

This variable exists in some versions. It has no meaning for the PLC project.

This variable is used by nearly all library function blocks to store a unique ID for the duration of their
execution. To this end the content that was found is stored in a local variable of the function block and
restored immediately before the function block is exited.

Should the program crash, or if there is a suspicion that there was a problem in a library function block, the
nDebugTag variables of all axes should be checked. If a value <> 0 is present, the function block was
affected by the crash, and the reason should be investigated. The numeric values used are listed in the
library under "Global constants". In addition, the contents of ST_TcHydAxRtData [} 141].sTopBlockName
should be determined. Usually, the name of the function block called directly by the application can be found
here.

FAQ #17: What has to be taken into account when Sercos drives are used?

If Sercos drives (from V3.0.26) are used, the following rules must be followed:

• The Sercos master interface (e.g. FC7501 etc.) must be allocated the name "SercosMaster" in the
System Manager. Otherwise neither control of the Sercos phase nor parameter and diagnostics
communication is possible.

• Only a Sercos segment with the library can be used.

• In the System Manager, the drive devices at the Sercos Segment should be allocated the name under
which they are known to the library by calling the MC_AxUtiStandardInit_BkPlcMc() function block.
Otherwise neither control of the Sercos phase nor parameter and diagnostics communication is
possible.

• The input variable SystemState [} 189] of the Sercos master interface should be linked for each drive

device of the Sercos segment.

• If one or several drives at the Sercos segment are reset, the segment can interrupt the fieldbus. In this
case, the Sercos master interface will undergo a corresponding phase change. Usually, the startup up
to phase 4 will be automatic. Then:

◦ the axis addressed by the reset will be error-free, as long as there are no ongoing problems.

332

Version: 1.8.3

TF5810

Knowledge Base

◦ all other axes at the Sercos segment will be in error state (fieldbus failure, axis not ready for
operation). Once the triggering reset of the first axis has been processed, the other axes can
usually be brought into an error-free state through a reset without a phase change.

This behavior is determined by characteristics of the Sercos fieldbus and cannot be influenced by the
library. It must be taken into account in the application in a suitable manner.

• Depending on certain parameter settings of the drive actuator, axis parameters are determined

automatically or have to be specified manually:

◦ S-0-0076, bits 0 to 2 specify the weighting type of the position data. Supported features:

◦ a) 0 0 1 translatory weighting:

S-0-0123 defines the rotation resolution (encoder-interpolation). The revolutional feed rate is
calculated from this number and the weighting (S-0-0077, S-0-0078).

◦ b) 0 1 0 rotary weighting:

S-0-0079 defines the rotation resolution (encoder-interpolation). The revolutional feed rate has to
be set manually.

◦ S-0-0044, bits 0 to 2 specify the weighting type of the velocity data. Supported features:

◦ a) 0 0 1 translatory weighting:

The velocity control value is converted to a velocity in encoder increments per time, based on the
revolutional feed rate and the rotation resolution. This information is offset against the velocity
resolution (S-0-0045, S-0-0046) and output.

◦ b) 0 1 0 rotational weighting

The velocity control value is converted to a speed based on the revolutional feed rate and output.

◦ S-0-0091 is converted with the method described above for velocity control values and used as
reference velocity. If the maximum speed exceeds the value determined in this way, it is limited
accordingly.

FAQ #18: How is a pressure or a force determined?

To determine an actual pressure or an actual force, one or several function blocks of types
MC_AxRtReadPressureDiff_BkPlcMc [} 220], MC_AxRtReadForceDiff_BkPlcMc [} 215],
MC_AxRtReadForceSingle_BkPlcMc [} 218] or MC_AxRtReadPressureSingle_BkPlcMc [} 222] have to be called
for each axis. Details for the call sequence can be found under FAQ #8 [} 329].

The AD converter values to be transferred to the function blocks have to be linked with allocated variables of
the application. Details regarding selection and parameterization can be found in the function blocks
descriptions.

FAQ #19: What has to be taken into account when AX5000 drives are used?

For AX5000 devices, when communication is established, a series of IDNs are read from the device and
various parameters are automatically calculated.

IDN
44

45
46
76
79
91

...used for parameters
Reference velocity, internal: Scaling of the velocity
output
internal: scaling of the velocity output
internal: scaling of the velocity output
Encoder interpolation
Encoder interpolation
Reference velocity

The following parameters are thus set automatically and cannot be influenced via the PlcMcManager:

TF5810

Version: 1.8.3

333

Knowledge Base

Parameter

Global: reference velocity

Encoder: inc. interpolation

Calculated from the maximum
speed of the device and the
revolutional feed rate.
Read from IDN79 of the device.

...influences which other param-
eters
Manuel velocities, max. appl. vel.

Note: The revolutional feed rate
has to be entered as inc.
evaluation.

FAQ #20: How do I prepare an axis for blending based on PLC Open?

In Hydraulik.lib it is possible to command up to 12 buffered movements. For this purpose, a command buffer
of type ST_TcPlcCmdCmdBuffer_BkPlcMc must be passed to the MC_AxUtiStandardInit_BkPlcMc [} 254]
function block for updating the axis reference and a function block MC_AxRtCmdBufferExecute_BkPlcMc
must be called cyclically.

If Move function blocks such as MC_MoveAbsolute_BkPlcMc [} 73], MC_MoveRelative_BkPlcMc [} 77] or
MC_MoveVelocity_BkPlcMc [} 79] are now activated, they enter their data in the command buffer.

In buffered mode make sure that the Move function blocks and the MC_AxRuntime_BkPlcMc [} 237] function
block of the axis run in a PLC task.

334

Version: 1.8.3

TF5810

Transition between a slow and a fast section.

Knowledge Base

TF5810

Version: 1.8.3

335

Knowledge Base

Transition between a fast and a slow section.

FAQ #21: How can I access registers of a terminal, to which an encoder or a valve of an axis is
connected?

For register communication with terminals to which the encoder or the valve of an axis is connected, it is
recommended to use function blocks of types MC_AxUtiReadRegDriveTerm_BkPlcMc [} 293](),
MC_AxUtiReadRegEncTerm_BkPlcMc [} 294](), MC_AxUtiWriteRegDriveTerm_BkPlcMc [} 302]() and
MC_AxUtiWriteRegEncTerm_BkPlcMc [} 303]().

FAQ #22: What is the structure of an ASCII file for a linearization table?

The format of an ASCII file from a linearization table is specified as follows:

• One linearization point per row.

• For each row first a velocity value, then an output value.

• The velocity values are normalized to the reference velocity. They are therefore in the range -1,000 to

1,000 inclusive.

• The output values are normalized to the full scale value. They therefore cover the range -1,000 to

1,000.

336

Version: 1.8.3

TF5810

Knowledge Base

• The first value in a row may be preceded by white space characters (space, tab).

• Between the two values in row there must be at least one white space character (space, tab).

• Between the two values of a row there may be further white space characters (space, tab).

• Point and comma are permitted as decimal separator.

• No non-digits are permitted between a negative sign and the first digit.

• The first point specifies the negative end of the table.

• The velocity value of all further points must be higher (i.e. less negative or more positive) than its

predecessor.

• It makes sense if the output value of a point is higher (i.e. less negative or more positive) than its

predecessor, since otherwise there would be a negative slope in this section. This would result in a
change of sign of the gain and therefore instability in an active control.

• The zero point (i.e. both coordinates of the point are 0.000) has to be specified.

Example: The following (idealized) table describes a cylinder, which in negative direction only reaches half
the velocity of the positive direction due to asymmetric effective areas (due to single-sided piston rod). It is
assumed that the cylinder is operated with a zero overlap valve with a bend in the characteristic curve at
40%

Normalized velocity
-0.500
-0.430
-0.360
-0.290
-0.220
-0.150
-0.080
-0.060
-0.040
-0.020
0.000
0.040
0.080
0.120
0.160
0.300
0.440
0.580
0.720
0.860
1.000

Normalized output
-1.000
-0.900
-0.800
-0.700
-0.600
-0.500
-0.400
-0.300
-0.200
-0.100
0.000
0.100
0.200
0.300
0.400
0.500
0.600
0.700
0.800
0.900
1.000

FAQ #23: How can PlcMcManager commands be blocked?

In some situations the triggering of commands by the PlcMcManager can be problematic. This would be the
case if a certain sequence of actions has to be processed completely, for example. In order to prevent
inadvertent issuing of commands by the PlcMcManager in such cases, the
MC_AxRtCommandsLocked_BkPlcMc [} 262] function can be used to enter a lock in the status double word of
the axis. If this lock is active, any command sent by PlcMcManager sent is rejected with a write protection
error.

It is essential to remove the lock, once the action to be protected has been processed. This also
and in particular applies in the event of errors.

TF5810

Version: 1.8.3

337

Knowledge Base

An example [} 378] is available.

FAQ #24: What format do files with characteristic curve data have?

If an axis is equipped with components for a characteristic curve-controlled linearization, the interpolation
points can be exported to a file using a function block of the type MC_LinTableExportToAsciiFile_BkPlcMc().
The function block MC_LinTableImportFromAsciiFile_BkPlcMc() is available if such data are to be imported
from a file created or modified in this way or in other ways.

So that such files can be used, the following format must be strictly adhered to.

• A row must be used for each point.

• Each row must be terminated with CR/LF.

• Each row initially contains the normalized velocity value in the range -1.0 to 1.0.

• After at least one separator (space or tab), the normalized output value follows for the full range from

-1.0 to 1.0.

• The output value must exhibit a constant step size (distance between two points).

• The first row contains the negative end value. Its output value must be -1.0.

• The last row contains the positive end value. Its output value must be 1.0.

• A row must be provided for the output value 0.0.

• The file must therefore contain the same number of points in the negative and positive range. The total

number must be odd.

• Both the velocity and the output value follow the same rules:

Sign with negative numbers

A digit

A decimal separator: comma or point

At least one decimal place

Optional: An exponent, marked with "e", a minus sign and a digit

Example: -1.81408951053528e-1 -5.0e-1

Example: 0.333 0.5

5.2

Global constants

Available from version 3.0

Bit-masks for position cams

These masks are to be used by the application to provide digital movement cams for bActPosCams in
ST_TcHydAxRtData.

Constant
bTcHydActPosCamPos
bTcHydActPosCamHigh
bTcHydActPosCamUp
bTcHydActPosCamDown
bTcHydActPosCamLow
bTcHydActPosCamNeg

Description
Summary of bTcHydActPosCamHigh and bTcHydActPosCamUp.
The axis has reached the upper target position.
The axis is located close to the upper target position.
The axis is located close to the lower target position.
The axis has reached the lower target position.
Summary of bTcHydActPosCamLow and bTcHydActPosCamDown.

Bit-masks for axis status information

These masks are to be used by the application to interrogate status signals in nStateDWord in
ST_TcHydAxRtData.

338

Version: 1.8.3

TF5810

Knowledge Base

Constant
dwTcHydNsDwFunctional
dwTcHydNsDwReferenced
dwTcHydNsDwSteady
dwTcHydNsDwInTargRng

dwTcHydNsDwInTarget

Description
Axis is ready for operation.
Axis has been referenced.
Axis is not active.
The axis is located within a distance from the target position
specified by fMonPositionRange in ST_TcHydAxParam.
The axis has been located without interruption since a time specified
by fMonTargetFilter within a distance from the target position
specified by fMonTargetRange in ST_TcHydAxParam.

dwTcHydNsDwDontTouchProtected Reserved. Not supported.
dwTcHydNsDwStopped

dwTcHydNsDwBusy
dwTcHydNsDwMoveUp
dwTcHydNsDwMoveDown
dwTcHydNsDwReferencing
dwTcHydNsDwConstVelo
dwTcHydNsDwExtSetpointActive

dwTcHydNsDwStartedOver

dwTcHydNsDwControlActive
dwTcHydNsDwErrState

The last movement of the axis was stopped without reaching the
specified target position.
The axis is active.
The axis is moving in the direction of increasing positions.
The axis is moving in the direction of decreasing positions.
Axis is homing.
The axis is moving with constant velocity.

The axis is controlled by an MC_AxRtSetExtGenValues_BkPlcMc
[} 252] function block.
The axis was started, i.e. the last accepted command took effect
while the axis was still in motion.
Reserved. Not supported.
The axis is in an error state.

Bit-masks for axis enable information

These masks are to be used by the application to provide enable signals in nDeCtrlDWord in
ST_TcHydAxRtData.

Constant
dwTcHydDcDwCtrlEnable

dwTcHydDcDwFdPosEna

dwTcHydDcDwCtrlPosEna

dwTcHydDcDwFdNegEna

dwTcHydDcDwCtrlNegEna

Description
Controller enable. This enable is a precondition for the output of
control value and controller outputs.
Advance movement enable in positive direction. This enable is a
precondition for the output of control value and controller outputs in
the direction of increasing values of position.
Combination of dwTcHydDcDwCtrlEnable and
dwTcHydDcDwFdPosEna.
Advance movement enable in negative direction. This enable is a
precondition for the output of control value and controller outputs in
the direction of decreasing values of position.
Combination of dwTcHydDcDwCtrlEnable and
dwTcHydDcDwFdNegEna.
Referencing cam.

dwTcHydDcDwRefIndex
dwTcHydDcDwAcceptBlockedDrive Reserved. Not supported.
dwTcHydDcDwBlockedDriveDetected Reserved. Not fully supported.

This signal suppresses any active velocity controller.

Error Codes

These constants are to be used for the outputs of ErrorID from function blocks and for nErrorCode in
ST_TcHydAxRtData.

TF5810

Version: 1.8.3

339

Knowledge Base

Constant

dwTcHydAdsErrNoError
dwTcHydAdsErrUnknownPo
rt

Hexadeci-
mal
0
16#0006

Decimal Description

0
6

No error.
ADS port unknown. Possible causes:

• AMS NetID / ADS port address the wrong runtime

system or the wrong computer

• another project is running in the addressed PLC

• the application does not call a

MC_AxAdsCommServer_BkPlcMc [} 279]() function
block

dwTcHydAdsErrUnknownTa
rget

dwTcHydAdsErrInvalidIdxGr
oup

16#0007

7

Target machine unknown. Possible causes:

• AMS NetID / ADS port address the wrong runtime

system or the wrong computer

• the target system has not been started

• TwinCAT has not been started

• the connection is electrically / mechanically

interrupted

• for communication via Ethernet: the TCP/IP

connection is not working

16#0702

1794

Invalid IndexGroup. Possible causes:

dwTcHydAdsErrInvalidIdxOf
fset

16#0703

1795

• AMS NetID / ADS port address the wrong runtime

system or the wrong computer

• another project is running in the addressed PLC

• application software error (incorrect combination of

ADS port / IdxGroup / IdxOffset)
Invalid IndexOffset. Possible causes:

• AMS NetID / ADS port address the wrong runtime

system or the wrong computer

• another project is running in the addressed PLC

• application software error (incorrect combination of

ADS port / IdxGroup / IdxOffset)

• attempted access to an array element with invalid

index (out of bounds)

• a write access to a variable without write

permission was requested

Size (number of bytes) not permitted. Possible
causes:

• application software error (incorrect combination of

ADS port / IdxGroup / IdxOffset)
Value not permitted. Possible causes:

• the transferred value is outside absolute parameter

limits

• the transferred value is outside parameter limits,

which have been specified by other already
applicable parameters

dwTcHydAdsErrRdWrNotPe
rmitted

16#0704

1796

Access (write, read) not permitted. Possible causes:

dwTcHydAdsErrInvalidSize 16#0705

1797

dwTcHydAdsErrIllegalValue 16#0706

1798

dwTcHydAdsErrNotReady

16#0707

1799

Not ready for operation. Possible causes:

• an MC_Power_BkPlcMc function block was

prompted by its Enable input to activate an axis
that is not ready for operation

340

Version: 1.8.3

TF5810

Constant

dwTcHydAdsErrBusy

Hexadeci-
mal
16#0708

Decimal Description

1800

Already active. Possible causes:

Knowledge Base

dwTcHydAdsErrNoFile
dwTcHydAdsErrSyntax

16#070C
16#070D

1804
1805

• the axis could not accept an instruction because it

is already dealing with another task
Reserved: File is missing / not accessible.
Syntax in command or file invalid. Possible causes:

• invalid characters or character combinations were
detected while reading a characteristic curve file
stored in ASCII format

• incomplete information was detected while reading
a characteristic curve file stored in ASCII format

dwTcHydAdsErrTimeout

16#0745

1861

Timeout. Possible causes:

• during a communication the response did not

arrive within a designed time

◦ the chosen time is too short

◦ the connection is interrupted

• the process has prevented processing of the

command or delayed it beyond the designated
time

• the specified commands parameters have
increased the time requirement beyond the
designated value

dwTcHydAdsErrNoAmsAddr 16#0749

1865

AMS/ADS address missing:

dwTcHydErrCdNotCompatib
le

16#4040

16448

dwTcHydErrCdIllegalOutput
Number

16#4104

16644

dwTcHydErrCdNotSupport 16#4107

16647

dwTcHydErrCdCycleTime

16#4205

16901

dwTcHydErrCdMissingEnc 16#4210

16912

• The ADS address of the device was not mapped to
the corresponding variable of the input structure.
The axis is incompatible with the required function.
Possible causes:

• application software error
The output number is outside the permitted range.
Possible causes:

• an MC_ReadDigitalOutput_BkPlcMc or

MC_WriteDigitalOutput_BkPlcMc function block
was called with an invalid parameter.

Function or command not supported. Possible
causes:

• application software error
Cycle time (fCycletime in ST_TcHydAxParam) not
permitted. Possible causes:

• Parameterization error
There is no connection to an encoder interface
(pStDeviceInput and/or pStDeviceOutput in
Axis_Ref_BkPlcMc [} 86]). Possible causes:

• Application software error (the

MC_AxUtiStandardInit_BkPlcMc function block
was not called or not provided with the address of
an ST_TcPlcDeviceInput and an
ST_TcPlcDeviceOutput structure)

TF5810

Version: 1.8.3

341

Knowledge Base

Constant

Hexadeci-
mal

Decimal Description

dwTcHydErrCdMissingDrive 16#4212

16914

dwTcHydErrCdCannotSync
hronize

16#421A

16922

dwTcHydErrCdIllegalGearF
actor

16#421B

16923

dwTcHydErrCdSoftEnd

16#4222

16930

16936
16953

16#4228
16#4239

dwTcHydErrCdLowDist
dwTcHydErrCdIllegalStartTy
pe
dwTcHydErrCdCommandBu
fferOverflow
dwTcHydErrCdEncLostCam
m
dwTcHydErrCdCtrlEnaLost 16#4260

16#4253

16#423F

dwTcHydErrCdEncNoCam
mFound
dwTcHydErrCdEncNoCam
mEnd
dwTcHydErrCdEncNoSync
Pulse
dwTcHydErrCdAcc
dwTcHydErrCdDec
dwTcHydErrCdJerk
dwTcHydErrCdPtrPlcMc

16#4309
16#430A
16#430B
16#4345

17161
17162
17163
17221

dwTcHydErrCdPtrMcPlc

16#4346

17222

dwTcHydErrCdCtrlEna
dwTcHydErrCdNegFdEna

16#4356
16#4357

17238
17239

There is no connection to a drive interface
(pStDeviceInput and/or pStDeviceOutput in
Axis_Ref_BkPlcMc [} 86]). Possible causes:

• Application software error (the

MC_AxUtiStandardInit_BkPlcMc function block
was not called or not provided with the address of
an ST_TcPlcDeviceInput and an
ST_TcPlcDeviceOutput structure)

Start distance inadequate when an
MC_GearInPos_BkPlcMc() function block is called.
Possible causes:

• the axis is too close to the sync point when the

function block is activated

• the dynamic axis parameters are inadequate
The parameters of a gear coupling are not permitted.
Possible causes:

• the parameter of the function block is not permitted
The target position is located on the far side of an
active software limit switch, and is therefore not
permitted.
The travel distance is unacceptably small.
Invalid start type.

16959

Command buffer is full.

16979

Reserved. Not supported.

16992

Controller enable was withdrawn during the motion.
Possible causes:

• the axis enable was withdrawn at an unexpected

time due to a machine logic signal

16#429C

17052

• application software error
Reserved. Not supported.

16#429D

17053

Reserved. Not supported.

16#429E

17054

Reserved. Not supported.

The acceleration is not acceptable.
The deceleration is not acceptable.
The jerk limitation is invalid.
No connection to one of the required axis interfaces
(pStDeviceInput or pStDeviceOutput in
Axis_Ref_BkPlcMc [} 86]).
No connection to one of the required axis interfaces
(pStDeviceInput or pStDeviceOutput in
Axis_Ref_BkPlcMc [} 86]).
Movement without controller enable is not permitted.
Movement in the direction of reducing positions
without the negative direction advance enable is not
permitted.

342

Version: 1.8.3

TF5810

Constant

dwTcHydErrCdPosFdEna

Hexadeci-
mal
16#4358

16#4359
dwTcHydErrCdSetVelo
dwTcHydErrCdPehTimeout 16#435C

16#435D
16#43A0

dwTcHydErrCdNotMoving
dwTcHydErrCdConsequenti
al
16#4401
dwTcHydErrCdEncType
16#4406
dwTcHydErrCdEncScaling
dwTcHydErrCdEncSyncDist 16#4414

dwTcHydErrCdEncSetActP
os
dwTcHydErrCdPtrPlcEncIn 16#4442

16#4422

16#4443

dwTcHydErrCdPtrPlcEncOu
t
dwTcHydErrCdEncUnderru
n
dwTcHydErrCdEncOverrun 16#4451

16#4450

dwTcHydErrCdPosLag
dwTcHydErrCdDriveType
dwTcHydErrCdRefVelo

16#4550
16#4601
16#4605

dwTcHydErrCdStepperStall
ed
dwTcHydErrCdPtrPlcDriveIn 16#4642

16#4636

16#4650

16#4643

16#4A02

dwTcHydErrCdPtrPlcDriveO
ut
dwTcHydErrCdDriveNotRea
dy
dwTcHydErrCdTblEntryCou
nt
dwTcHydErrCdTblInvalidMa
sterStep
dwTcHydErrCdTblNoInit
dwTcHydErrCdTblIllegalInd
ex
dwTcHydErrCdTblLineCoun
t
dwTcHydErrCdNotStartable 16#4B01
dwTcHydErrCdFuncTimeout 16#4B07

16#4A10
16#4A13

16#4A04

16#4A15

Decimal Description

Knowledge Base

17240

17241
17244

17245
17312

17409
17414
17428

17442

17474

17475

17488

17489

17744
17921
17925

17974

17986

17987

18000

Movement in the direction of increasing positions
without positive direction advance enable is not
permitted.
The required velocity is not acceptable.
The axis does not reach the target window within the
specified time.
The axis is not moving, or not in the correct direction.
Consequential error: The axis was put in an error
state due to a problem with another axis.
The parameter type is invalid.
The increment scaling is not permitted.
The distance between Latch_Enable and the sync
pulse is too small.
A problem occurred during actual value setting.

The axis does not have a pointer to an encoder input
interface
The axis does not have a pointer to an encoder
output interface.
Reported by some encoder types: The actual position
has passed the lower count limit of the encoder.
Reported by some encoder types: The actual position
has passed the upper count limit of the encoder.
Drive actuator or encoder report a hardware fault.

An error was detected when operating an SSI
encoder.
The lag error exceeds an active limit.
The value set in nDrive_Type is not permitted.
Reference velocity (fRefVelo in ST_TcHydAxParam)
is invalid.
A stall situation was detected.

The axis does not have a pointer to a drive input
interface.
The axis does not have a pointer to a drive output
interface.
Power section not ready for operation.

18946

The number of table entries (rows) is not permitted.

18948

18960
18963

The table contains entries with invalid master step
size.
The table is not initialized.
Table index not permitted.

18965

The number of table entries is too large.

19201
19207

Axis in a state that does not allow it to start.
The function was not reported as complete within the
specified time.
The axis is not in an operable state.

dwTcHydErrCdEncHdwFail
ed
dwTcHydErrCdSsi

16#4464

17508

16#4470

17520

dwTcHydErrCdNotReady

16#4B09

19209

TF5810

Version: 1.8.3

343

Knowledge Base

Constant

Hexadeci-
mal

Decimal Description

dwTcHydErrCdHomingType 16#4F00

20224

dwTcHydErrCdEncCutOff

16#4F01

20225

dwTcHydErrCdIllegalDistan
ce
dwTcHydErrEncDisconecte
d

dwTcHydErrDriveDisconect
ed

Referencing method (nEnc_HomingType in
ST_TcHydAxParam) is not permitted.
The limit frequency for the actual value acquisition
has been exceeded.
Distance is invalid: zero or negative.

16#4F02

20226

16#4FF0

20464

Encoder hardware is uncoupled. Possible causes:

• the fieldbus connection is interrupted

• the power supply for the device is not available

• the device is faulty

• another device, which is located in the fieldbus

connection between the controller and the device,
has no power supply or is faulty

16#4FF1

20465

Drive hardware is uncoupled. Possible causes:

• the fieldbus connection is interrupted

• the power supply for the device is not available

• the device is faulty

• another device, which is located in the fieldbus

connection between the controller and the device,
has no power supply or is faulty

dwTcHydErrDistanceInsuffic
ient
dwTcHydErrIllegalAreas

16#4FF2

20466

The travel path is inadequate.

16#4FF3

20467

Inadmissible effective areas:

dwTcHydErrIncompleteImpl
ementation

• inadmissible values have been entered for the

cylinder areas on the valve tab

• the combination of the registered areas is not

permitted in this way

16#4FF4

20468

The axis implementation is incomplete:

• although the axis is marked with bDriveIsHybrid in

its parameters, no
MC_AxRtHybridAxisActuals_BkPlcMc function
block is called

• the same instance of type ST_TcPlcDeviceInput is

also transferred to the
MC_AxUtiStandardInit_BkPlcMc function block of
another axis

• the same instance of type ST_TcPlcDeviceOutput

is also transferred to the
MC_AxUtiStandardInit_BkPlcMc function block of
another axis

• another instance of ST_TcPlcMcLogBuffer is

passed to another axis

• the same valid pointer to an instance of

ST_TcPlcCmdBuffer_BkPlcMc was passed to
another axis

• the same valid pointer to an instance of

ST_TcMcAutoIdent was passed to another axis

Device-specific error codes of function block MC_Power_BkPlcMc

These values appear at the ErrorID output of an MC_Power_BkPlcMc function block, if an error is reported
by the external device.

344

Version: 1.8.3

TF5810

Constant

dwTcHydErrCdAX2000Main
PwrTmOut

Hexadeci-
mal
16#0001

dwTcHydErrCdAX2000Main
PwrFault
dwTcHydErrCdAX2000Pwr
StageTmOut

16#0002

16#0003

dwTcHydErrCdAX2000Pwr
StageFault
dwTcHydErrCdAX2000Rep
ortsError

16#0004

16#0005

dwTcHydErrCdAX2000Error
I2T
dwTcHydErrCdAX2000Error
Chopper
dwTcHydErrCdAX2000Error
WatchDog

16#0006

16#0007

16#0008

1

2

3

4

5

6

7

8

9

dwTcHydErrCdAX2000Error
PwrLine
dwTcHydErrCdAX2000Con
nectionLost

dwTcHydErrCdAX2000Con
nectionTmOut
dwTcHydErrCdKL2531Over
Temp
dwTcHydErrCdKL2531Unde
rVoltage

dwTcHydErrCdKL2531Ope
nLoadA
dwTcHydErrCdKL2531Ope
nLoadB
dwTcHydErrCdKL2531Over
CurrentA
dwTcHydErrCdKL2531Over
CurrentB
dwTcHydErrCdKL2531NotR
eady
dwTcHydErrCdKL2531Conn
ectionLost

16#0009

16#000A

10

16#000B

11

16#0001

16#0002

16#0003
16#0004

16#0005

16#0006

16#0007

16#0008

1

2

3
4

5

6

7

8

16#000A

10

dwTcHydErrCdKL2531Conn
ectionTmOut

16#000B

11

ADS Codes

Decimal Description

Knowledge Base

Only for AX2000: no feedback by the mains contactor
(timeout during waiting for
ST_TcPlcMcAx2000In.bPowerOk).
Only for AX2000: negative edge on feedback from
mains contactor (ST_TcPlcMcAx2000In.bPowerOk).
Only for AX2000: no feedback from AX output stage
(timeout during waiting for
ST_TcPlcMcAx2000In.DriveState[3].6, no Ready).
Only for AX2000: Negative edge of AX output stage
(ST_TcPlcMcAx2000In.DriveState[3].6, no Ready).
Only for AX2000: error message from AX device
(ST_TcPlcMcAx2000In.DriveState[3].7 or
ST_TcPlcMcAx2000In.DriveError<>0).
Only for AX2000: I2T error message from AX output
stage (ST_TcPlcMcAx2000In.DriveState[0].0).
Only for AX2000: brake resistor of the AX output
stage faulty (ST_TcPlcMcAx2000In.DriveState[0].1).
Only for AX2000: watchdog (timeout during
communication) of the AX output stage was triggered
(ST_TcPlcMcAx2000In.DriveState[0].3).
Only for AX2000: supply error reported by AX output
stage (ST_TcPlcMcAx2000In.DriveState[0].4).
Only for AX2000: The connection to the AX device is
broken or substantially disrupted
(ST_TcPlcMcAx2000In.BoxState<>0).
Only for AX2000: The communication with the AX
device could not be established (timeout).
Only for KL2531/KL2541: The KL2531/KL2541
terminal reports overtemperature alarm.
Only for KL2531/KL2541: The KL2531/KL2541
terminal reports inadequate supply voltage on the
power rail.
Only for KL2531/KL2541: Reserved.
Only for KL2531/KL2541: The KL2531/KL2541
terminal reports broken wire on the A-side.
Only for KL2531/KL2541: The KL2531/KL2541
terminal reports broken wire on the B-side.
Only for KL2531/KL2541: The KL2531/KL2541
terminal reports overcurrent at output stage A.
Only for KL2531/KL2541: The KL2531/KL2541
terminal reports overcurrent at output stage B.
Only for KL2531/KL2541: The terminal reports a
output stage problem (enabled, not ready).
Only for KL2531/KL2541: The connection to the
terminal is broken or substantially disrupted
(ST_TcPlcMcDriveIn.uiBoxState<>0).
Only for KL2531/KL2541: The communication with
the terminal could not be established (timeout).

These constants are accepted by the MC_AxAdsReadDecoder and MC_AxAdsWriteDecoder function
blocks.

TF5810

Version: 1.8.3

345

Knowledge Base

IndexGroup
16#4000 + axis
index

IndexOffset
2

Type
STRING()

R/W Description
R

Axis name in text form.

4
16#10003

UDINT
UDINT

16#10006

LREAL

16#30003

16#4100 + axis
index

1

16#10002

16#10005

16#4200 + axis
index

1

16#10
16#21
16#FFFF0001
16#FFFF0002
16#81

16#4300 + axis
index

UDINT

UDINT

LREAL

LREAL

-

-
Structure
-
-
UDINT

16#B1

UDINT

16#F000 + axis
index

1

2

Structure

R
R

R

R

R

R

R

W

W
W
W
W
R

R

R

Cycle time in microseconds.
Encoder type: nEnc_Type from
ST_TcHydAxParam.
Incremental evaluation:
fEnc_IncWeighting from
ST_TcHydAxParam.
Drive type: nDrive_Type from
ST_TcHydAxParam.
Error code: nErrorCode from
ST_TcHydAxRtData.
Actual position: fActPos from
ST_TcHydAxRtData.
Actual velocity: fActVelo from
ST_TcHydAxRtData.
Execute axis reset.

Start homing.
Start axis movement.
Save parameters.
Load parameters.
Status double word: nStateDWord
from ST_TcHydAxRtData.
Error code: nErrorCode from
ST_TcHydAxRtData.
The ST_TcHydAxRtData variable
for the axis.

Structure

R/W The ST_TcHydAxParam variable

16#800F0000 + axis
index

16#FFFFFFFF

E_TcMCParameter
[} 104]
0
1
2
3
4

String()
UINT
UINT
UINT
UINT

Array Dimensions

for the axis.

R/W Parameters and actual values of

the axis.

R
R
R
R
R

Identification of the server.
Major version of the library.
Minor version of the library.
Release of the library.
Number of axes supported

The following constants used for dimensioning of fields and can be used by the application.

346

Version: 1.8.3

TF5810

Knowledge Base

Description

Lower boundary index on an array[] of CAMSWITCH_REF_BkPlcMc
[} 123], supplied to blocks of type MC_DigitalCamSwitch_BkPlcMc
[} 54]

Upper boundary index on an array[] of CAMSWITCH_REF_BkPlcMc
[} 123], supplied to blocks of type MC_DigitalCamSwitch_BkPlcMc
[} 54]

Lower boundary index on an array[] of TRACK_REF_BkPlcMc [} 126],
supplied to blocks of type MC_DigitalCamSwitch_BkPlcMc [} 54]

Upper boundary index on an array[] of TRACK_REF_BkPlcMc [} 126],
supplied to blocks of type MC_DigitalCamSwitch_BkPlcMc [} 54]

Constant
ciBkPlcMc_CamSwitchRef_MinIdx

ciBkPlcMc_CamSwitchRef_MaxIdx

ciBkPlcMc_TrackRef_MinIdx

ciBkPlcMc_TrackRef_MaxIdx

Logger Levels

The following constants are used for the specification of the level, from which messages are included in the
logger function of the library.

Constant
dwTcHydLogLevel_None
dwTcHydLogLevel_Errors
dwTcHydLogLevel_Warnings
dwTcHydLogLevel_Actions

Logger Sources

Description
No logging
Only error messages
Error messages and warnings
Error messages, warnings and activities

The following constants are used to specify the source of messages in the logger function of the library.

Constant
dwTcHydLogSource_Library
dwTcHydLogSource_LibExt_2R2V
dwTcHydLogSource_Application
dwTcHydLogSource_ApplicationFramework

Description
A function block of the hydraulics library
A function block of the 2R2V library
A function block of the application
A function block of an application platform

Logger Argument Types

The following constants are used to specify the type of an optional parameter for a message in the logger
function of the library.

Constant
dwTcHydLogArgType_DInt

dwTcHydLogArgType_LReal

dwTcHydLogArgType_String

5.3

Valve

Description
The message contains a parameter of type DINT. The message text
must include a placeholder in the form %d.
The message contains a parameter of type LREAL. The message
text must include a placeholder in the form %f.
The message contains a parameter of type STRING. The message
text must include a placeholder in the form %s.

The valve is generally the actuator, which controls the axis. For continuous valves, a distinction is made
between:

• Servo valve

• Proportional valve

• Control valve

TF5810

Version: 1.8.3

347

Knowledge Base

Servo valve

These valves control large oil flows via small electrical signals

• A small torque motor controls the connected control oil, thereby adjusting the slider of the main stage.

• Often multi-stage design

• High responsiveness and controllability

Proportional valve

A coil current generates a proportional force, which moves the valve slider against the force of a spring.

Compared to servo valve:

• Longer step response time

• Higher current consumption

• Larger hysteresis

• More robust against contamination

• Attractive price

Control valve:

A proportional valve, for which the slider position is measured and automatically adjusted:

• Shorter step response time

• Smaller hysteresis

• Smaller load reaction

• More complex and more expensive than proportional valves

• Electronics on the valve or in the control cabinet

Basic principles of reading valve data sheets

A continuous valve is usually used as actuator for a controller. The designs of valves from different
manufacturers or different types may differ quite significantly. In order to adapt the output scaling to the
particular situation, the valve data sheet for the continuous valve must be available during commissioning. A
valve has a number of hydraulic ports. A and B are the valve outputs; A is connected to the cylinder side with
the larger piston area, B is connected to the cylinder side with the smaller piston area. P and T represent the
supply connections. P is the pressure line, and T is the return line to the tank.

In the hydraulics library, the A-side is always the side under positive pressure, the B-side is the side
under negative pressure.

In many cases the valve slide has to move slightly before an oil flow can be detected. This stroke is listed in
the valve data sheet under overlap.

The data sheet may indicate an overlapped valve, although this overlap is compensated in the valve
electronics.

348

Version: 1.8.3

TF5810

Knowledge Base

The characteristic volume flow curve shows the key information for the valve. The diagram above shows that
the piston itself has an overlap of 20%, which was reduced to 5% in the valve electronics. As a result, no
overlap compensation via the hydraulics library is required.

The fact that overlap compensation was carried out in the valve does not make it a zero overlap
valve, and the axis is therefore only capable of position control to a limited degree.

The diagram shows that the oil flow in the A-chamber of the piston is greater than the oil flow in the B-
chamber. This asymmetry indicates an area compensation in valve, in this case with a ratio of 11:6.

5.4

Electric/hydraulic hybrid axes

Hybrid axis concepts

The manufacturers of hydraulic components have developed a number of solutions for the design of such an
axis. In order not to have to create a dedicated solution for each model of each manufacturer, generalized
concepts were developed that combine a group of comparable models. The trailing letters (a, b, c, ...) denote
equivalent variants of a concept. In the following section, these concepts and variants are presented using
samples. The list of these samples is, by its very nature, incomplete.

The screen contents shown below are only visible if the 'hybrid' flag is set on the Valve tab.

Simplified representation

The concepts listed below require the use of various pressure limiters and anti-cavitation check valves for
their safe and long-term function. These components are indispensable but have no direct influence on the
basic function of the axis. For a better understanding, all circuits are simplified to a greater or lesser degree
and should not be regarded as documentation of an actual product.

TF5810

Version: 1.8.3

349

Knowledge Base

1: Simulation of a synchronous cylinder without regenerative circuit

The control behavior of a synchronous cylinder is simulated using a hydraulic synchronous or differential
cylinder and an adapted pump arrangement. A gear shift can be realized by a pump changeover, although
this results in a different concept.

1a: Simulation of a synchronous cylinder with a differential cylinder

Manufacturer / Product: Voith Turbo / CLDP.

Here, two pumps with flow rates adapted to the cylinder areas are operated on a motor shaft. The flow rates
of the pumps Q1 / Q2 must be adapted to the ratio of the effective areas. Since this is not always perfectly
possible, complex pressure distributions can result, depending on the situation.

The control results in the behavior of a synchronous cylinder with direction-independent feed constant.

Hydraulically a differential cylinder is present and an exchange volume is to be stored.

No gear shift is available. It can be realized by synchronous flow-switchable pumps. This results in a different
concept.

350

Version: 1.8.3

TF5810

Knowledge Base

Required parameters: Effective area in positive direction (1), effective area in negative direction (2), volume
per revolution at the effective area in positive direction (3), maximum pump speed (4).

Automatically calculated parameters: Volume per revolution at the effective area in negative direction. The
ratio of the effective areas is used.

Automatically set parameters: The selectable areas are 0, the rotation volumes for force mode are equal to
the values for rapid mode.

TF5810

Version: 1.8.3

351

Knowledge Base

1b: Simulated synchronous cylinder with multiple differential cylinders

Manufacturer / Product: Bucher / Demo HMI2018.

Here, three differential cylinders with an area ratio of 2:3 are mechanically connected in parallel. The three
smaller areas A2abc are hydraulically connected in parallel and form an effective area. Of the larger areas,
A1ab are hydraulically connected in parallel, while the third area A1c is ventilated.

For control purposes, the result is a compound synchronous cylinder with direction-independent feed
constant.

Hydraulically a synchronous cylinder is present and no exchange volume is to be stored.

According to the manufacturer a gear shift is possible. This results in a different concept.

352

Version: 1.8.3

TF5810

Knowledge Base

Required parameters: Effective area in positive direction (1), effective area in negative direction (2), volume
per revolution at the effective area in positive direction (3), maximum pump speed (4).

Automatically calculated parameters: Volume per revolution at the effective area in negative direction. The
ratio of the effective areas is used (in this case 1:1).

Automatically set parameters: The selectable areas are 0, the rotation volumes for force mode are equal to
the values for rapid mode.

2: Simulation of a synchronous cylinder with regenerative circuit

The control behavior of a synchronous cylinder is simulated using a hydraulic differential cylinder and an
adapted pump arrangement in a regenerative circuit. A gear shift can be realized by a pump changeover,
although this results in a different concept.

TF5810

Version: 1.8.3

353

Knowledge Base

2a: Simulation of a synchronous cylinder with regenerative circuit

Manufacturer / Product: Bosch Rexroth / application.

Here, two pumps with flow rates adapted to the cylinder areas are operated on a motor shaft. The flow rates
of pumps Q1 / Q2 must be adapted to the area ratio of rod cross-section / ring area. Since this is not always
perfectly possible, complex pressure distributions can result, depending on the situation.

The control results in the behavior of a synchronous cylinder with direction-independent feed constant.

Hydraulically a differential cylinder is present and an exchange volume is to be stored.

No gear shift is available. It can be realized by synchronous flow-switchable pumps. This results in a different
concept.

The oil volume in Q1 and the volume of lines through which only their flow rate flows must be
smaller than the oil exchanged during operation for the cross-section of the piston rod. Otherwise
there is no safe oil exchange.

354

Version: 1.8.3

TF5810

Knowledge Base

Required parameters: Effective area in positive direction (1), effective area in negative direction (2), volume
per revolution at the effective area in positive direction (3), maximum pump speed (4), maximum pump
speed (5).

Automatically calculated parameters: Volume per revolution at the effective area in negative direction. The
ratio of the effective areas is used.

Automatically set parameters: The selectable areas are 0, the rotation volumes for force mode are equal to
the values for rapid mode. The flag for regenerative operation is set.

3: Gear shift through switching of effective areas

Switching valves are used to make the effective areas of a cylinder effective or ineffective or to connect them
in a variable manner. In some cases, 'virtual' areas are created which have to be taken into account in the oil
quantity but do not contribute to force build-up.

TF5810

Version: 1.8.3

355

Knowledge Base

3a: Use of a cylinder with 2+1 active areas

Manufacturer / Product: EH-D.

Rapid traverse switching: V1 and V2 (note valve symbol!) are switched off. V3 must be switched to extend
the cylinder. If A1 and A3 are the same, no oil is exchanged with the reservoir. Otherwise, differential oil
must be taken in via the RV or displaced via the DBV, depending on the direction.

Force mode: V1 and V2 (note valve symbol!) are switched on. During extending, activation of V3 is
mandatory. The oil quantity from A3 is supplemented for A1/A2 via the RV. Retracting is only possible in this
configuration by displacing a considerable volume via the DBV. Heat is generated during this process. This
combination of valve switching and direction of rotation of the pump is useful for pressure reduction, but it
should not be used for active movement.

Required parameters: Active area in extending direction = A1 (1), added active area in extending direction =
A2 (2), active area in retraction direction = A3 (3), added active area in retraction direction = 0 (4), volume
per revolution at active area in positive direction (5), maximum pump speed (6).

Automatically calculated parameters: The volume per revolution on the active area in negative direction is
set equal to the volume per revolution on the effective area in positive direction.

Automatically set parameters: The rotation volumes for force mode are equal to the values for rapid mode.

356

Version: 1.8.3

TF5810

3b: Virtual area switching generation

Knowledge Base

Manufacturer / Product: Voith Turbo / CLSP.

In this case, the valves produce a gear shift.

Rapid mode:

During extending, V1a and V2b are activated. The oil quantity for the ring area is exchanged via V1a
between the areas. The oil quantity for the rod cross-section is supplemented via V2b by the pump from the
reservoir. Depending on the circuit, the rod cross-section is hydraulically supported. The cylinder has a low
natural frequency and should be operated with adapted dynamics.

During retracting, V1b and V2a are activated. The oil quantity for the ring area is exchanged via V1b by the
pump between the areas. The oil quantity for the rod cross-section is diverted to the reservoir via V2a. Due
to the circuit, area A2 is only subjected to the pre-load pressure of the reservoir. The cylinder is only to some
extent able to brake by its own force. It should be operated with adapted dynamics.

Force mode:

During extending, V1b and V2b are activated. The oil quantity for the ring area is exchanged via V1b by the
pump between the areas. The oil quantity for the rod cross-section is supplemented via V2b by the pump
from the reservoir. Due to the circuit, area A2 is only subjected to the pre-load pressure of the reservoir. The
cylinder is only to some extent able to brake by its own force. It should be supported and slowed down by the
process.

During retracting, V1b and V2b are activated. The oil quantity for the ring area is exchanged via V1b by the
pump between the areas. The oil quantity for the rod cross-section is discharged via V2b through the pump
to the reservoir. Due to the circuit, area A2 is only subjected to the pre-load pressure of the reservoir. The
cylinder is only to some extent able to move by its own force. It must be checked whether it is able to
overcome the forces generated by gravity and friction. This circuit should only be used to reduce forces
generated in extending direction.

TF5810

Version: 1.8.3

357

Knowledge Base

Required parameters: Effective area in extending direction = rod cross-section (1), added effective area in
extending direction = ring area (2), effective area in retraction direction = ring area (3), added effective area
in retraction direction = 0 (4), volume per revolution at effective area in positive direction (5), maximum pump
speed (6).

Automatically calculated parameters: The volume per revolution on the effective area in negative direction is
set equal to the volume per revolution on the effective area in positive direction.

Automatically set parameters: The rotation volumes for force mode are equal to the values for rapid mode.

358

Version: 1.8.3

TF5810

3c: Virtual area switching generation

Knowledge Base

Manufacturer / Product: EH-D / 18-0129-001-HY-K.

In this case, the valves produce a gear shift.

Rapid mode:

During extending, activation of V1 is optional, activation of V2 and V3 is mandatory. The oil quantity for the
ring area is exchanged via V3 / V1 between the areas. The oil quantity for the rod cross-section is added via
V2 from the reservoir. Due to the circuit, area A1 is only subjected to the pre-load pressure of the reservoir.
Due to the circuit, area A2 is only subjected to the pre-load pressure of the reservoir. The cylinder is only to
some extent able to move by its own force. It must be checked whether it is able to overcome the forces
generated by gravity and friction. This circuit should only be used to reduce forces generated in extending
direction.

During retracting, activation of V1 and V2 is mandatory, activation of V3 is optional. The oil quantity for the
ring area is exchanged via V1 / V3 by the pump between the areas. The oil quantity for the rod cross-section
is diverted to the reservoir via V2. Due to the circuit, area A2 is only subjected to the pre-load pressure of the
reservoir. The cylinder is only to some extent able to brake by its own force. It should be operated with
adapted dynamics.

Force mode:

During extending, activation of V1 is optional, activation of V3 is mandatory, activation of V2 is prohibited.
The oil quantity for the ring area is exchanged via V3 / V1 by the pump between the areas. The oil quantity
for the rod cross-section is supplemented via RV by the pump from the reservoir. Due to the circuit, area A2
is only subjected to the pre-load pressure of the reservoir. The cylinder is only to some extent able to brake
by its own force. It should be supported and slowed down by the process.

TF5810

Version: 1.8.3

359

Knowledge Base

During retracting, activation of V1 is mandatory, activation of V3 is optional, activation of V2 is prohibited.
The oil quantity for the ring area is exchanged via V1 / V3 by the pump between the areas. The oil quantity
for the rod cross-section is diverted to the reservoir via DBV. Due to the circuit, area A2 is only subjected to
the limiting pressure of the DBV. The cylinder is only to some extent able to move by its own force. It must be
checked whether it is able to overcome the forces generated by gravity and friction.

Required parameters: Effective area in retraction and extension direction = ring area (1, 3), added effective
area in retraction and extension direction = rod cross-section (2, 4), volume per revolution at the effective
area in positive direction (5). The added effective area in the extension direction must be marked as 'virtual'
(7), since it must be taken into account when calculating the required speed, but does not contribute to the
force build-up, maximum pump speed (6).

Automatically calculated parameters: The volume per revolution on the effective area in negative direction is
set equal to the volume per revolution on the effective area in positive direction.

Automatically set parameters: The rotation volumes for force mode are equal to the values for rapid mode.

360

Version: 1.8.3

TF5810

3d: Virtual area switching generation

Knowledge Base

Manufacturer / Product: Voith Turbo / PDSC.

In this case, the valves produce a gear shift.

Rapid mode:

During extending and retracting, activation of V1 is mandatory. Part of the oil quantity from A2 is used
regeneratively for A1, the remaining quantity is exchanged via pump with A3. Since A3=A1-A2, the behavior
is synchronous, both hydraulically and from a control perspective.

Force mode:

During extending, activation of V1 is prohibited. Since the pressure in A1 is higher than the pressures in A2
and A3 due to the effect of the pump, the RV locks and the DBV connects A2 and A3. Since A1=A2+A3, the
behavior is synchronous, both hydraulically and from a control perspective.

Retraction in force mode is not possible since the RV generates the rapid mode configuration with hydraulic
control.

TF5810

Version: 1.8.3

361

Knowledge Base

Required parameters: Effective area in retraction and extension direction = A3 (1, 3), added effective area in
retraction and extension direction = A2 (2, 4), volume per revolution at A1 (5), maximum pump speed (6).

Automatically calculated parameters: The volume per revolution at A2+A3 is set equal to the volume per
revolution at the effective area in positive direction.

Automatically set parameters: The rotation volumes for force mode are equal to the values for rapid mode.

3e: Virtual area switching generation

362

Version: 1.8.3

TF5810

Knowledge Base

Manufacturer / Product: EH-D / ECO.

In this case, the valves produce a gear shift.

Rapid mode:

During retracting, activation of VA1 and VA2 is mandatory, activation of VB2 is optional. Activation of VB1
and VA3 is prohibited. The pump conveys oil from A1 to A2; the portion originating from A2 for the rod cross-
section is discharged via VA1 to the reservoir.

During extending, activation of VA1 and VB2 is prohibited. Activation of VB1 and VA2 is optional. Activation
of VA3 is mandatory. The oil quantity from A2 is used regeneratively via VA3 for A1, the remaining quantity
is added via pump and VB1 from the reservoir.

Force mode:

During extending, activation of VA1 and VA3 is prohibited. Activation of VB1 and VA2 is optional. Activation
of VA2 is mandatory. The oil quantity from A2 is used regeneratively via VB2 for A1, the remaining quantity
is added via pump and VB1 from the reservoir.

Retracting in force mode is not possible because the DBV would have to discharge the oil quantity for the
rod cross-section at high pressure.

For decompression of area A1, the pressure can be reduced in force mode, while activation of VB1 and VA2
is mandatory.

Required parameters: Effective area in retraction direction = ring area (1), added effective area in retraction
direction = 0 (2), effective area in extension direction = rod cross-section (3), added effective area in
extension direction = ring area (4), volume per revolution at A1 (5), maximum pump speed (6).

Automatically calculated parameters: The volume per revolution at A2 is set equal to the volume per
revolution at the effective area in positive direction.

Automatically set parameters: The rotation volumes for force mode are equal to the values for rapid mode.

5.5

Configuration of an axis

In contrast to the Beckhoff NC, the axis in the hydraulic library is configured by the application itself. This
means that the function blocks for operating an axis (read actual value, generate setpoints, generate position
rules, linearization and output) must be called up individually.

TF5810

Version: 1.8.3

363

Knowledge Base

All function blocks work on a common axis reference, which must be created globally. If there is more than
one axis, the axis references must be created as an array.

In addition to the axis reference (AXIS_REF_BkPlcMc [} 86]), the I/O structures ST_TcPlcDeviceInput [} 149]
and ST_TcPlcDeviceOutput [} 153] must be declared for each axis. Further optional elements are added,
depending on the application.

To view messages a ST_TcPlcMcLogBuffer [} 156] should be declared. This buffer is shared by all axes.

If other sensors such as pressure or load cells are used in the application in addition to position detection,
the I/O value must be set in the application. The parameterization of the scaling can be managed in the
fCustomerData[] section of the axis. For each axis 20 customer-specific data are provided in this section.
This data is saved via the axis, loaded and displayed in the PLcMcManager. For the display in the
PlcMcManager the label can be changed by declaring the structure ST_TcMcAuxDataLabels [} 149].

Sample for the data of an axis

General settings

An attribute must be set in TwinCAT 3 so that the I/O is always read in with a constant time interval,
regardless of the time required by the program.

In TwinCAT 2, the I/O flag at the start of the task must be set in the System Manager under PLC
configuration.

364

Version: 1.8.3

TF5810

Knowledge Base

In contrast to NC, the hydraulic axis itself (setpoint generator, controller, etc.) is calculated directly in the
PLC. It is therefore recommended to set the cycle time of the task to less than 10 ms.

Initialization

The PLCopen standard specifies that all Motion function blocks of the application are called with an instance
of type AXIS_REF_BkPlcMc. For technical reasons, some axis components cannot be contained in such an
instance, since they must be located in separate areas (e.g. process images). Other elements are optional
and are only be added if required. To link them to the axis reference, they are transferred to an initialization
function block of type MC_AxUtiStandardInit_BkPlcMc [} 254].

When called for the first time, the function block links the input and output structures and all optional
elements with the axis reference. Variables that have to be passed as addresses are marked with the prefix
"p". The function block should be called cyclically to check the pointer addresses.

It is not permitted to bind an instance of ST_TcPlcDeviceInput, ST_TcPlcDeviceOutput or
ST_TcMcAutoUdent to multiple axes. It is not permitted to connect more than one instance of
ST_TcMcLogBuffer to axes.

The function block loads the parameters from the given file path and transfers them to the axis reference. All
parameters are stored in binary form in an Axis name.dat file.

Once the parameters have been loaded successfully, the bParamsEnable flag in the axis reference becomes
TRUE. Only now is the use of parameters that have not yet been defined ruled out, and all other axis-related
function blocks may be called.

TF5810

Version: 1.8.3

365

Knowledge Base

Actual value acquisition

The encoder type set in the parameter structure of the axis reference determines how and from which
variables of the input structure the MC_AxRtEncoder_BkPlcMc [} 198] function block will read the actual value
and convert it to a position [mm] and a velocity [mm/s]. The connection is monitored when EtherCAT
components are used.

If the actual values are very noisy, it is possible to filter them via a sliding average value
(MC_AxUtiSlidingAverage_BkPlcMc [} 269]) or a Pt1 element (MC_AxUtiPT1_BkPlcMc [} 267]). The use of
custom filters is possible.

Filter function blocks must be called after the encoder function block. The variable to be filtered must be
passed to their input. The result can be written back to the corresponding variable of the axis reference. This
causes the old noisy value to be replaced by a new, stabilized value.

If a heavily filtered actual value is used for control purposes, the dynamics and controllability can be
affected due to the filter jump response.

Additional function blocks are available for reading in pressure and force values. The function block to be
used depends on the variable to be measured. In contrast to position determination, for force and pressure
determination the mapping interface and terminal monitoring must be provided by the application.

MC_AxRtHybridAxisActuals_BkPlcMc [} 224] is an adapted function block for determining the
essential actual values of a servo-electric/hydraulic hybrid axis.

Setpoint generation and default position controller

If, for example, MC_MoveAbsolute_BkPlcMc [} 73] triggers an active movement of the axis, the setpoint
generator calculates the current values for the set velocity and the set position in each cycle. This can be
done in a time-controlled or path-controlled manner. Permanent position control is required for time-
controlled generation, otherwise this is only required at standstill. Several profile variants are supported. For
more information, please refer to the documentation for the function block.

If the axis does not have a command buffer, a command is entered directly in the runtime data of the axis.
Otherwise, commands are buffered, subjected to path planning, and then made effective according to the
blending rules.

If required, the application can handle the setpoint generation. An MC_AxRtSetExtGenValues_BkPlcMc [} 252]
function block must be used for this purpose. If external generation is active, the library block to be called is
switched to a passive state and then reactivated. In this way, application-specific gear units and other non-
standard mechanisms can be realized.

The setpoint generator and a default position controller that is adequate in most cases are integrated in the
MC_AxRuntime_BkPlcMc [} 237] function block.

366

Version: 1.8.3

TF5810

MC_AxRuntime_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ErrorUDINT  ErrorIDBOOL  LagErrorAlternative position controller

If another controller is called after the default position controller and fLagCtrlOutput is overwritten in the
runtime data of the axis reference, another position controller can be activated. This can be a customer-
specific controller or another controller from the library such as the FB MC_AxRtPosPiControllerEx_BkPlcMc
[} 185].

Knowledge Base

This library controller is a PID controller with optional extensions such as condition feedback and
acceleration pre-control.

Further controllers

Pressure or force controllers are used in many applications with hydraulic axes. As an example, an
MC_AxCtrlPressure_BkPlcMc [} 172] function block is shown here.

In the active state, the function block overwrites the output of the setpoint generator. In order for the
controller response to take effect, it must be called up before linearization.

When activating or deactivating, step changes in the control values of the axis can occur depending
on the parameter values.

Final processing

At this point, the control values of the axis are present in a form that assumes linear behavior of the axis and
its components. In practice, this is rarely the case. To take this into account, the control values (setpoints,
controller outputs, overlap compensation) are combined to an output value and subjected to linearization.
This adjustment can be carried out in sections or based on characteristic curves.

Sectional linearization

The library provides the function block MC_AxRtFinish_BkPlcMc [} 246] for simple linearization.

The set velocity weighted with the pre-control and the controller output are added to the output velocity.

An active overlap compensation is selected such that it is ramped linearly from zero to the set overlap
compensation Ovl between 0 and VCreep. It is fully effective for the remaining area.

TF5810

Version: 1.8.3

367

MC_AxRtPosPiControllerEx_BkPlcMcReset  BOOLI_Enable  BOOL↔Axis  Reference To AXIS_REF_BkPlcMcLREAL  SetPosLREAL  SetVeloLREAL  ResponseBOOL  InWindupMC_AxCtrlPressure_BkPlcMcEnable  BOOLReset  BOOLFirstAuxParamIdx  INTkP  LREALTn  LREALReadingMode  E_TcMcPressureReadingModePreSet  LREALWindupLimit  LREALAlignAreas  BOOL↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ErrorUDINT  ErrorIDBOOL  InWindupMC_AxRtFinish_BkPlcMc↔Axis  Reference To AXIS_REF_BkPlcMcBOOL  ErrorUDINT  ErrorIDKnowledge Base

The direction dependency is compensated. The output velocity is multiplied by fAreaRatio from the axis
parameters if the velocity is positive and fAreaRatio ≥ 1.0. If the velocity is negative and fAreaRatio ≤ 1.0
division is applied.

The output is formed by adding the weighted target velocity, the controller output, the active overlap
compensation and the offset correction.

Linearization based on characteristic curve

The library provides the function block MC_AxRtFinishLinear_BkPlcMc [} 247] for this linearization with higher
resolution.

If the use of the characteristic curve is not enabled or not possible, an internal function block is used for
sectional linearization. This is the case if at least one of the following reasons applies:

• FALSE is transferred at the enable input of the function block.

• No instance of type ST_TcMcAutoIdent has been linked to the axis reference.

• bLinTabAvailable in the axis parameters is FALSE: The characteristic curve is not valid.

Otherwise, the target velocity weighted with the pre-control and the controller output are added to the output
velocity. The two characteristic curve points closest to the calculated value are determined and the output
value is formed by intermediate interpolation and addition of the offset correction.

368

Version: 1.8.3

TF5810

Characteristic curve measurement

Knowledge Base

The function block MC_AxUtiAutoIdent_BkPlcMc supports the measurement of a characteristic curve by
means of a standardized automatic sequence. The parameters to be set for this are stored in the structure
ST_TcMcAutoIdent [} 128]. If a characteristic curve measurement and a characteristic curve-based
linearization are to be used, such an element must be created and connected to the axis reference.

An MC_AxUtiAutoIdent_BkPlcMc function block must be called after the
MC_AxRtFinishLinear_BkPlcMc [} 247] function block and before the MC_AxRtDrive_BkPlcMc [} 187]
function block of the axis.

The characteristic curve determined in this way combines the influences of a number of sources:

• Non-linearities of the valve

• Asymmetry of the cylinder

• Flow effects at higher velocities

• Possible limitations due to a pump

• Positional influences such as gravitation

• Influences of other components in the oil flow

With a servo-electric/hydraulic hybrid axis, no MC_AxUtiAutoIdent_BkPlcMc function block may be
activated.

Output adjustment

At this point, the control values for the axis are available as physical or standardized parameters. Only the
MC_AxRtDrive_BkPlcMc [} 187] function block determines an output parameter that represents these
parameters in a form that is converted to the desired response by the device used. The method used and its
parameters are set in the parameter structure of the axis reference.

Interfacing of the PlcMcManager

In preparation.

The PlcMcManager is connected via the TwinCAT ADS service. Since this allows only one port per
application, all axes must use a common connection. Multiple instances of this function block are not
permitted.

TF5810

Version: 1.8.3

369

Knowledge Base

The sample shown applies to an application with only one axis. Multi-axis projects must combine the axis
references in an array whose address and first and last index are transferred.

This FB must be called independently of whether axes can load their parameters.

Message logging

All axes of an application share a logging buffer. To send the messages that arrive there to the Event Log of
the operating system and, if available, to the message window of the development environment, create an
instance of the function block MC_AxRtLoggerSpool_BkPlcMc [} 259] for each application. The call of the
function block is independent of whether axes can load their parameters.

5.5.1

FB_Power

The function block manages the axis enables. A distinction is made between controller enable and direction-
dependent feed enable in positive and negative direction. Feed enable is an internal enable for the setpoint
generator, whereas controller enable is used for the position controller and also for the output stage of
drives.

5.6

The PlcMcManager

The PlcMcManager supports commissioning and testing of axes, which are automated using the hydraulics
library. It visualizes the actual state and enables access to parameters and triggering of commands.

The PlcMcManager is not intended for operating machines and systems. It is not a substitute for a
user interface.

For your safety

 WARNING
Risk of injury due to unexpected machine behavior!

The commands triggered by the PlcMcManager can obstruct automatic actions and responses of the
control software obstruct or influence them in an unexpected or undesirable direction. This may result in
unexpected and dangerous movements.

• Make sure that neither you nor others are harmed by the movement, e.g. by maintaining a suitable

safety distance.

• Do not carry out any action whose consequences you cannot assess.

Installation

For TwinCAT 2: A license-free copy of the PlcMcManager is provided with the library or the documentation.
Select a suitable path, then create a shortcut on the desktop of the PC. Without such a shortcut, the
PlcMcManager can only be started from Explorer.

370

Version: 1.8.3

TF5810

For TwinCAT 3: When downloading the library, a license-free copy of the PlcMcManager is created in the
directory C:\TwinCAT\Functions\TF5810-TC3_Hydraulics-Positioning. If your TwinCAT not installed under to
C: or in another directory, the path must be adjusted accordingly.

Knowledge Base

Running the PlcMcManager

If the tool is stored on the PC, it can be started by double-clicking.

Offline display of a parameter file

In the menu bar under Online you will find the Offline file mode, where a dialog for selecting an axis
parameter file of type DAT is offered. When a file is opened, the axis parameters are show like in online
mode, as far as possible.

No actual axis states are shown, and no axis commands can be triggered. This also applies if the
displayed parameters belong to an axis, to which access would be possible.

Online operation

If the runtime system with the library function blocks is not present on the PC on which the PlcMcManager is
running, the target system has to be selected first. In the menu bar under Online you will find the Target
dialog, where the computers are listed that are entered as Remote Computers in TwinCAT System
Service on the AMS Router tab.

By selecting a Remote Computer, the communication with the runtime system is activated automatically. If
the runtime system with the library function blocks is present on the PC on which the PlcMcManager is
running, the communication with the runtime system can be activated with Login via the menu bar under
Online.

In the current versions the PlcMcManager is prepared for use under TwinCAT 3. To establish the connection
at runtime, it checks the expected ADS addresses for both TwinCAT 2 and TwinCAT 3. This may take
several seconds, particularly if a network connection is used. The details shown below should then appear.

TF5810

Version: 1.8.3

371

Knowledge Base

1. Shows the port and the server used for the communication with the runtime system.
2. The mode is displayed. Since no axis has been selected up to this point, the PlcMcManager is still in

OFFLINE mode.

3. Shows the version information of the library used by the PLC application.

If these details do not appear after a few seconds, the connection has failed. This can have a number of
reasons:

• No target system was selected, despite the fact that the application is not running on the same

computer as the PlcMcManager.

• The PLC application does not contain a MC_AxAxAdsCommServer_BkPlcMc [} 279] function block or

does not call it.

• The application is not running on the selected target system.

• No connection to the selected target system.

• The PC on which the PlcMcManager is running has no access rights to the selected target system.

• The PLC is not running.

If a dialog with an error message appears at this point, the connection to the target system is disturbed
(timeout), or the PlcMcManager and the library used in the application are not compatible. Incompatibility is
usually due to a new library version being used, without updating the PlcMcManager.

Many parameter input fields have a "?" field on the left-hand side. This can be used to call up a brief
explanation of the parameter.

Sample: Explanation of the parameter <Global.CreepDistance>:

372

Version: 1.8.3

TF5810

Knowledge Base

First steps

Double-clicking on the server shown on the left displays the axes used in the application as a list. Click on an
axis to select it. Its status is then cyclically updated, and its parameter are accessible. If the communication
fails for some reason, it can be restarted by clicking on an axis.

This example shows the file path and name used for this axis. However, an InitError 1804 (0x70C) and an
InitState of -2 are reported. The error code indicates a file error and the InitState is "negative terminated".
There are several possible causes for this:

• The path does not exist on the computer where the PLC application is running. Problems can easily

arise if the application goes online for the first time on another system.

TF5810

Version: 1.8.3

373

Knowledge Base

• The path is not accessible from the location of the PLC runtime. This is possible, for example, if the

path points to a network.

• Reading and/or writing is not allowed on this path.

• The path or file name is not spelled correctly. The backslash may be missing at the end of the path

name.

• There is no corresponding file under the specified path name.

The last cause listed always occurs when commissioning of a PLC application is started without an existing
file. To create a file with default parameters, press the [Save] key to initiate a write operation with the initial
parameter values. The [Reset] key deletes the error state, and in this case the loading of the parameters
from the file is repeated. If the problem cannot be solved by this procedure, it is caused by another of the
listed causes.

Data and commands

The PlcMcManager only graphically displays variables from the PLC. Runtime values can be found in the
AxisRef in stRtData. Parameters that are changed via the PlcMcManager must actively be written to the
variables of the PLC via the Activate button. All values that have to be saved permanently are stored in the
AxisRef under stAxParams. These parameters are saved by the PLC, not by the PlcMcManager.

If the axis is controller and feed enabled by the PLC with an MC_Power_BkPlcMc function block, it can be
moved using the jog keys (<, <<, >>, >). At this time it is still a simulated axis. The axis can also be
commanded via the Position and Velocity fields. The movement command is executed via the Start button.

5.7

Sample programs

Available from version 3.0

Structure of the application

The application is largely made up of PLCopen function blocks. A selection of function blocks is available,
which are equipped with an interface defined by the PLCopen. A number of examples are described below,
which provide a good basis for project configuration.

Each example contains the project file, the required axis parameter files and a scope configuration. The axis
parameter files must be stored in a folder on the target system. The file path must be adjusted in the global
constant "cnst_ParamFilePath" of the project file.

Example 1: Single axis

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
1599853451.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
4937463179.zip

The MC_AxUtiStandardInit_BkPlcMc [} 254] function block loads the parameters and monitors the pointer
addresses. After the data has been loaded successfully, "bParamsEnable" becomes TRUE and the actual
axis blocks are called.

MC_AxStandardBody_BkPlcMc [} 253] internally calls the required function blocks such as
MC_AxRtEncoder_BkPlcMc [} 198], MC_AxRuntime_BkPlcMc [} 237], MC_AxRtFinish_BkPlcMc [} 246] and
MC_AxRtDrive_BkPlcMc [} 187]. However, if a filter, a pressure regulator, a characteristic curve measurement
or similar is required, the individual components must be called instead of MC_AxStandardBody_BkPlcMc
[} 253].
By using a MC_AxAdsCommServer_BkPlcMc [} 279] function block the axis can be commanded via the
PlcMcManager. The MC_AxParamDelayedSave_BkPlcMc function block saves changes made by the
PlcMcManager after a given time (here 10 s).
Via the PlcMcManager you can log onto the target system and actively move the axis.

374

Version: 1.8.3

TF5810

Knowledge Base

Example 2: Multi-axis application

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
1599855627.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
4937465739.zip

The example illustrates a configuration with arrays of function blocks and structures. The range of functions
corresponds to example 1.

Example 3: Pressure-controlled braking

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
1599857803.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
4937468299.zip

The example shows how the pressure regulator MC_AxCtrlSlowDownOnPressure_BkPlcMc [} 178] throttles
the feed rate of an axis depending on the pressure. In this example, the controller becomes active when the
actual pressure exceeds the set pressure. Since the result is transferred via an application code to
"fLagCtrlOutput", the controller must be called after the setpoint generator. Otherwise, fLagCtrlOutput would
be overwritten by the position controller in MC_AxRuntime_BkPlcMc [} 237].
If a command is started in the PlcMcManager with a velocity of 100 mm/s and a position of 500 mm, for
example, the scope shows that the pressure increases continuously with increasing position. At a position of
400 mm, the system has reached the set pressure of 50 bar and stops.

Example 5: Move function blocks

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
1599859979.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
4937547659.zip

In this example, various function blocks are used for motion control. If the variable bStart becomes TRUE,
the state machine starts the axis with MC_MoveAbsolute_BkPlcMc [} 73] to the position 500 mm. When the
axis has reached the target and the target window conditions are met (in PosRang, in TargetRange for
TargetFilterTime and in BrakeDistance), a MC_MoveVelocity_BkPlcMc [} 79] automatically starts with a
velocity of 400 mm/s. This velocity remains active for 5 seconds and is then terminated with
MC_Stop_BkPlcMc [} 82], so that the axis comes to a standstill. This is followed by a relative movement of
100 mm with MC_MoveRelative_BkPlcMc [} 77] and a move to position 0.0 mm. Different acceleration and
deceleration ramps are used in the different motion profiles.

Example 6: Time ramp generator

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
1599862155.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
4937550219.zip

An axis without encoder cannot be controlled via the standard setpoint generator. For this type of axis,
iTcMc_ProfileTimeRamp [} 241] provides an alternative setpoint generator. If the variable "bUp" or "bDown" is
TRUE in the global variables, the axis moves at the specified velocity (here 500 mm/s) to the first limit switch
(DigCamP – for positive/ DigCamM – for negative) and then slows down to the corresponding creep velocity.
After reaching DigCamPP – for positive/ DigCamMM – for negative the output is deleted.

Example 7: Override and function generator

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
1599864331.zip

TF5810

Version: 1.8.3

375

Knowledge Base

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
4937552779.zip

Demonstration of the function block MC_SetOverride_BkPlcMc [} 41]. Global variables (bOverrideSinusoidal,
fOverrideCycleTime, fOverrideMinValue, fOverrideMaxValue) can be used to specify the sequence, the
period and the limitations of a signal generator, which modifies the override. Function blocks of type
MC_FunctionGeneratorFD_BkPlcMc [} 226], MC_FunctionGeneratorTB_BkPlcMc [} 228] and
MC_FunctionGeneratorSetFrq_BkPlcMc [} 227] are used for generating the override.

Example 8: Digital cam controller

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
1599866507.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
4937555339.zip

The example shows how to control digital cams through an axis and MC_DigitalCamSwitch_BkPlcMc [} 54].
In the example two cams are activated in TRACK_REF_BkPlcMc [} 126] (maximum 32). The first cam is
activated under three different conditions:
1. from position -1000 mm to 1000 mm and positive direction
2. from position 2000 mm to 3000 mm and positive direction
3. from position 3000 mm to 2500 mm and negative direction
The second cam has only one condition:
1. to be active in positive and negative direction for a time of 1.35 s from position 3000 mm.
In addition to the switching conditions, a cam can also have a switch-on and switch-off delay. For cam 1, the
switch-on delay is set to 0.125 s and the switch-off delay is set to 0.250 s. The conditions for switching a cam
are specified in CAMSWITCH_REF_BkPlcMc [} 123]. The output of a cam is specified in OUTPUT_REF_BkPlcMc
[} 126].
The axis must be commanded via the PlcMcManger (position greater than 3000 mm).

Example 9: Joystick

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
1599868683.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
4937557899.zip

The example demonstrates the use of the function block MC_MoveJoySticked_BkPlcMc [} 75]. With this
function block, the axis is moved in an endless motion at a velocity specified by JoyStick. Joystick is a
normalized value between +/-1.0, which, multiplied by the commanded velocity, results in the set velocity.

Example 10: Identification and linearization

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
1599870859.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
4937611659.zip

The example describes the automatic characteristic curve measurement with MC_AxUtiAutoIdent_BkPlcMc
and the use of the characteristic curve with MC_AxRtFinishLinear_BkPlcMc [} 247]. The settings for the
automatic characteristic curve measurement are accessible in the PlcMcManger under the LinDef tab and
can be found in the structure ST_TcMcAutoIdent [} 128].
In the example, you can choose between three different valve simulations using the global variable nTest. A
suitable .dat file is loaded according to the selected simulation. The parameters for the characteristic curve
measurement are preset in the .dat file as required. Note: If nTest is switched while the PLC is running, the
PlcMcManager must be reconnected. The following scenarios can be selected via nTest:

1. Only the overlap and velocity ratio is missing
2. A zero overlap characteristic curve with bend is missing

376

Version: 1.8.3

TF5810

Knowledge Base

3. A characteristic curve with overlap is missing

The variable "bStartAuto" can be used to start MC_AxUtiAutoIdent_BkPlcMc. During the measurement, the
function block returns a busy and the already measured characteristic curve is displayed on the LinTab tab.
If the measurement was successful, the characteristic curve can be used by the function block
MC_AxRtFinishLinear_BkPlcMc [} 247]. The characteristic curve is automatically saved and loaded in the .dat
file of the axis. The function block MC_AxTableToAsciFile_BkPlcMc [} 234] is available for exporting the
characteristic curve in an ASCII-readable format.

Example 11: Stop function blocks

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
1599873035.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
4937614219.zip

The different ways of stopping an axis are compared here. The example can be started by setting the
variable bStart to TRUE.

MC_Stop_BkPlcMc [} 82]: Executes a stop with preset deceleration parameters. The axis reports ready when
the calculated target including target tolerances (in PosRange, in TargetRange for target filter time and in
BrakeDistance) has been reached.

MC_EmergencyStop_BkPlcMc [} 57]: Brakes with preset ramp to standstill.

MC_ImediateStop_BkPlcMc [} 72]: Sets the set value to zero without ramp.

Example 12: Buffering and blending

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
1599875211.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
4937616779.zip

The basic procedure for buffered movements is explained in FAQ 20 [} 334]. To start the example, the
variable bStart must become TRUE. The Scope View shows that there are six movements, which are
processed in coupled mode.

Example 13: Filter

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
1599877387.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
4937619339.zip

The example shows the behavior of several filter types and what to consider when using filters.
If all signals with the name "Noisy" are switched off in Scope View, the original signal and the filtered signals
can be seen with corresponding offsets. The shape of the signal is retained. The more a signal is filtered, the
stronger the phase shift between the original and filtered signal. This phase shift has a direct influence on the
controllability of axes and other sections.
If the noisy signals are made visible in the Scope, it can be seen that the noise portion in the signal is
considerably lower both through a MC_AxUtiSlidingAverage_BkPlcMc [} 269] and after a
MC_AxUtiPT1_BkPlcMc [} 267].

Example 14: Function generator

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
1599879563.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
4937621899.zip

TF5810

Version: 1.8.3

377

Knowledge Base

In some applications, a setpoint generator is required to generate sinusoidal, trapezoidal or sawtooth signals.
For example, the signals generated with MC_FunctionGeneratorTB_BkPlcMc [} 228] and
MC_FunctionGeneratorFD_BkPlcMc [} 226] can be transferred to an axis via
MC_AxRtSetExtGenValues_BkPlcMc [} 252].

Example 15: Pressure regulator

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
1599881739.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
4937624459.zip

The example shows the reading and scaling of an actual pressure value in the application. A pressure
control for an axis with MC_AxCtrlPressure_BkPlcMc [} 172] is demonstrated.
The application first moves to a position at which a pressure increase is expected via a fast movement. The
movement continues at a slower velocity and the controller is activated when the set pressure has been
reached.

Example 16: Distributed axis references

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
1599883915.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
4937627019.zip

The example shows the use of a list of POINTER TO Axis_Ref_BkPlcMc. The use of
MC_AxAdsPtrArrCommServer_BkPlcMc [} 281] instead of MC_AxAdsCommServer_BkPlcMc [} 279] makes it
possible to distribute the axis references.

The list must be updated in each cycle. This update must be carried out before calling
MC_AxAdsPtrArrCommServer_BkPlcMc [} 281].

Sample 17: External setpoint generation

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
6407024139.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
9326778891.zip

The simple sample shows the basic use of a function block of the type MC_AxRtSetGenValues [} 252].

Example 18: Locking PlcMcManager

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
1599886091.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
4937629579.zip

It may be necessary to disable PlcMcManager commands such as Jog, MoveAbs or Stop. This can be done
in the PLC with MC_AxRtCommandsLocked_BkPlcMc [} 262].

Sample 19: External setpoint generation

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
9326087819.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
9326781195.zip

Here, a larger project demonstrates the possibilities of the external setpoint generator.

378

Version: 1.8.3

TF5810

Knowledge Base

Example 100: Electronic gearing

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
1599888267.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
4937663627.zip

The example shows how two slave axes can be coupled by an electronic gearing via a master axis (axis 3).
The coupling is created and released by MC_GearIn_BkPlcMc [} 63] and MC_GearOut_BkPlcMc [} 67].
It must be ensured that the dynamic parameters of the master and slave are compatible with each other,
otherwise the slave cannot follow the master.
To establish the coupling, the master and slave must be in idle state. The coupling can be released during
the motion. The master axis moves to the target and the slave axis is stopped when the coupling is released.

Example 101: Electronic cam plate

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
1599890443.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
4937666187.zip

Axes 1 and 2 are coupled to virtual axis 3 via a cam plate. In this example, the coupling parameters for axis
1 are stored in the text file TcPlcMcEx_101_2.txt. For axis 2, the coupling parameters are calculated in
function block "FB_CalculateCamTable2". MC_CamTableSelect_BkPlcMc [} 53] is used to specify the master
and slave axis and the cam table. In function block MC_CamIn_BkPlcMc [} 49] the coupling is generated and
the set values for the slave are calculated. If the master axis is moved via the PlcMcManager, the slave axis
follows the corresponding cam plate. The coupling is canceled with MC_CamOut_BkPlcMc [} 51].

Example 103: Flying gear coupling

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
1599892619.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
4937668747.zip

Demonstration of an activated flying gear coupling with function blocks MC_GearInPos_BkPlcMc [} 65] and
MC_GearOut_BkPlcMc [} 67].

Example 104: Synchronization control

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
1599894795.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
4937671307.zip

Demonstration of a synchronization control for a two-axis gantry using a virtual master. Synchronization
control is always used where two or more axes have to be controlled in a balanced manner. A virtual master
axis is used for generating the set values. The set values are distributed to the slave axes, which add their
local position controller. For example, the current position of the virtual master axis is calculated as an
average value over the slave axes.
In order to ensure smooth commissioning, it is essential that certain parameters are kept the same. This
applies in some cases within the group of slave axes, partly also for the master axis. In "FB_Parameter" this
is forced by cyclic copying.

Example 105: Linearization for synchronization control

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
1599896971.zip

TF5810

Version: 1.8.3

379

Knowledge Base

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
4937673867.zip

This example demonstrates the characteristic curve determination for a two-axis gantry (see also example
104) with the function blocks MC_AxUtiAutoIdent_BkPlcMc and MC_AxUtiAutoIdentSlave_BkPlcMc.

Sample 106: Flying coupling

For TwinCAT 2: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
9326092427.zip

For TwinCAT 3: https://infosys.beckhoff.com/content/1033/tf5810_tc3_hydraulic_positioning/Resources/
9326783499.zip

Some of the possibilities offered by the flying coupling and uncoupling of an electronic gear unit are
illustrated here.

5.8

Commissioning

The procedure described here refers to basic commissioning of an axis of which nothing is known. With
identical axes, certain points can be skipped.

5.8.1

Basic settings

In order to start up the real axis, various default settings must be applied.

The corresponding encoder type must be entered in the General tab. To do this, the corresponding encoder
must be selected via the selection menu and written to the runtime variables via Activate. The currently
active type is displayed to the left of the selection window.

The Knowledge Base contains a table [} 324], which helps to select the correct encoder type and explains
the mapping interface to I/O.

If, for technical reasons, it is not possible to determine the actual position with the standard encoder function
block of the library, this task can also be executed by application function blocks. Then enter the result in
fActPos and fActVelo in ST_TcHydAxRtData and update the position change in the current cycle in
fActPosDelta. bEncoderResponse should be used to indicate whether the actual values could be updated.
For the sake of uniformity use should again be made here of the fEnc_IncWeighting, fEnc_IncInterpolation
and fEnc_ZeroShift or fEnc_RefShift parameters.

380

Version: 1.8.3

TF5810

Knowledge Base

A range of devices and equipment might be functioning as actuators (Drivetyp), applying a variety of physical
principles to create a variable velocity that depends on an electrical magnitude. Depending on the
corresponding I/O component, the Drivetype must be set in the selection window and the variables must be
linked to the field device. The Knowledge Base contains a table [} 327] which supports the selection of the
type to be set.

If the position measuring system is an incremental system, the corresponding referencing method [} 103]
must also be defined.

On the Global tab you should initially enter 100 for the reference velocity. The value is corrected later, but in
this way, overlap etc. can be entered directly in %.

The acceleration and deceleration should be set to 100 mm/s². With this setting, this axis will accelerate to
reference velocity in 1 s. The jog parameters should be set to 5 mm/s and 10 mm/s. The creep velocity
should be set to 5 mm/sec, the creep distance should be 10 mm and the braking distance 2 mm.

If the valve is covered and the valve data sheet is available, you can enter the overlap from the data sheet
on the Valve tab.

On the Encoder tab, enter the resolution per increment in Inc. evaluation. Alternatively, an increment
number can also be specified in Inc. interpolation and the corresponding path in Inc. evaluation.

In the Controller tab, the lag and velocity controller must be set to zero.

For further commissioning, a Scope with the following variables should be recorded:

• SetVelo

• ActVelo

• SetPos

• ActPos

• fOutput

• fLagCtrlOutput

If available, record pressures, forces and valve slide position.

If the controller enable and feed enable of the axis are set, the axis must not move. If this is the case, a
temporary zero balance must be carried out.

5.8.2

Temporary zero compensation

The Offset compensation parameter is set in the Controller tab. Depending on the direction in which the
axis is drifting, a value between -10 V and +10 V must be entered. As a rule, values of +/- 0.5 V are to be
expected.

5.8.3

Movement directions

The jog button should be used to move the axis slowly. If this is not the case, the pressure supply must be
checked. Furthermore, switching valves may also have to be operated or the compensation of the valve
overlap is set too small.

It is recommended to specify a positive direction of movement for the axis that corresponds to the way the
machine works. If the axis moves in this direction, the actual position should count upwards. If this is not the
case, the counting direction can be inverted on the Encoder tab. If the direction of change of the indicated
position corresponds to the mechanical movement, but the direction of action of the given commands is not
as desired, the output can be inverted on the Valve tab.

When the valve output is inverted, the offset compensation must be adjusted, as it is not inverted
and its effect is reversed.

TF5810

Version: 1.8.3

381

Knowledge Base

5.8.4

Position measuring system

The axis should show a plausible actual position for both an absolute and an incremental position measuring
system. The zero point of the encoder and the defined zero point of the axis usually do not coincide. On the
Encoder tab, you can enter the desired current position and transfer it to the axis via the Set-Pos button. At
this point in time, this set position does not have to match the actual position exactly. Especially with
incremental measuring systems, homing is carried out later on.

The PlcMcManager adapts the display of the parameters as far as possible to the set encoder type. As a
result, different parameters can be visible for different axes.

For incremental encoder types, the diagram shown above appears. The visibility of the parameters for
homing depends on the set homing method.

To avoid collisions during commissioning, the software limit switches should be activated and set
appropriately in the Monitor tab. Since the actual position can differ slightly from the actual position, it is
recommended to set the software limit switches a little closer.

5.8.5

Characteristic curve measurement

The characteristic curve measurement (MC_AxUtiAutoIdent_BkPlcMc) not only determines the characteristic
curve itself, but also the reference velocity, the velocity ratio and the optional travel distance limits. For more
information on the setting options, see the function block itself.

The reference velocity should be preset to an approximate plausible value. One possibility is to calculate the
smaller cylinder area (A [mm²]) with the nominal volume flow (Qn [l/min)] of the valve:

Vref:= Qn*1.000.000/60/ A

The LinDef tab can be used to implement various settings. Further information can be found here.

382

Version: 1.8.3

TF5810

Knowledge Base

If this is activated, the AutoIdent function block starts by first determining the travel limits. The axis is then
positioned at a distance of at least 10 % from the travel limits, in order to determine the overlap. Once this
has been carried out successfully, the axis moves to the lower end and starts measuring. Depending on the
available travel path, several measurements are carried out in each direction.

Once the characteristic curve has been successfully measured, it can be viewed in the LinTab tab. A
successfully measured characteristic curve can be recognized by the fact that stParams.bLinTabAvaiable is
TRUE.

The chapter Coverage and reference velocity should be skipped if the characteristic curve was measured
successfully.

5.8.6

Overlap

In order to determine the overlap, the set velocity must be increased slowly until a response by the actual
velocity can be recognized. It is possible that the set velocity must be increased to a value of up to 30 mm/s
before a response of the actual velocity can be seen. When measuring the overlap, the overlap itself should
always be set to zero.

If different velocity set values are required in order to move the axis in positive or negative direction from
standstill, this indicates an asymmetric valve. In this case the check mark Asym in the Global tab must be
set and activated. The valve can now be parameterized separately in positive and negative direction.

The set velocity at which the axis moves must be entered under Overlap in the "Valve" tab. If the overlap has
already been assigned a value, this value must be taken into account. For asymmetric valves ensure that the
entry is made in the correct field; the overlap for the positive direction is expected in the upper field, the
overlap for the negative direction in the lower field.

After this optimization the axis should also respond at different small velocities. Whether the axis responds
with the right velocity is not important.

If an overlap has been entered from the data sheet and the axis always moves too fast, the overlap should
be reduced.

5.8.7

Reference velocity/velocity ratio

This chapter describes manual commissioning. A characteristic curve measurement also
determines the parameters discussed here. If it is used, this chapter should be skipped.

Once the axis can be moved at low velocity, the reference velocity must be set.

In order to determine the reference velocity, the set velocity is increased step-by-step, and a check is carried
out to determine whether the axis follows with approximately the set velocity.

In this step, only movements in the faster direction are to be evaluated. The oil is transported into
the small piston surface! The next step deals with directional dependency.

To trigger the required movements, the position and velocity can be specified in the Status tab. The
movement is executed with the Start button. The previously created Scope View should be used to analyze
the velocities.

The software limit switches should be activated and set so that the axis does not hit the mechanical
limit stops.

TF5810

Version: 1.8.3

383

Knowledge Base

If the actual velocity is much lower than the set velocity, the reference velocity should be reduced.

If the actual velocity is much higher than the set velocity, the reference velocity should be increased.

The appropriate reference velocity has been found when the medium to high set and actual velocities almost
match.

The reference velocity does not have to correspond to the actual or calculated maximum velocity of
the axis.

The following diagram shows the linearization section-by-section through overlapping and reference velocity
with a non-linear characteristic curve. It is left to the user to decide where the maximum deviation between
the linearization and the actual characteristic curve can occur.

384

Version: 1.8.3

TF5810

Knowledge Base

The usual asymmetry of the cylinders causes the axis to move too slowly in the slower direction at any
commanded velocity when the reference velocity is set. This behavior can be compensated for on the Valve
tab by using the velocity ratio parameter.

When the behavior is symmetrical, this parameter should be set to 1,000. If the positive direction of travel is
the slower direction, use a value greater than 1,000. If the negative direction of travel is the slower direction,
a value less than 1,000 should be used. This increases the output in the slower direction and compensates
for the asymmetry.

With this compensation, the output can only be increased up to its maximum value. The
parameterization must be carried out at velocities that the axis can reach in both directions.

If the parameter is changed in the wrong direction, the velocity decreases in the faster direction. In
this case the reference velocity must not be corrected.

5.8.8

Referencing

For incremental position measuring systems: Now at the latest, the axis should be referenced correctly and
fully. Enter the index velocity, index direction, sync velocity, sync direction and the reference position under
the Encoder tab. For more information see MC_Home_BkPlcMc [} 68].

It may be necessary to reset the travel limits.

5.8.9

Dynamics/target approach

At this point in time, the axis is able to position with different velocities and moderate dynamics.

On the Monitor tab you can set when the axis should report ready. An axis is in the target if the remaining
distance is smaller than PosRange and BrakeDistance; for the TargetFilterTime the remaining distance must
be smaller than Targetrange. These three parameters must be set appropriately according to the application
requirements.

TF5810

Version: 1.8.3

385

Knowledge Base

The user subsequently has to decide whether the axis should be positioned time-based or displacement-
based.

Most hydraulic applications can be operated path-controlled. If, however, time-based profile generation is
necessary, the TimeBased check mark should be set.

5.8.9.1

Displacement-based axis

The position controller is only active for the target approach.
The acceleration can be set so steeply that the axis gently accelerates without significant jerks when it starts
moving.
For braking on the target approach, not only the deceleration but also the creep distance, creep velocity and
braking distance must be set. All three parameters depend on each other and influence the target approach.
If the axis is within the braking distance, it is only controlled by the position controller. The creep velocity and
creep distance are used to stabilize the axis after deceleration, in order to take it to its target via the position
controller.

The target approach should look like this:

It is often observed that an axis that is extremely slowed down requires a longer creep phase in order to
position as accurately as an axis with a gentler deceleration.

5.8.9.2

Time-based axis control

If the axis control is to be time-based, the position controller is active during the entire motion. This option
should only be used for axes with a high natural frequency and ideally with a zero overlap valve.

The acceleration must be limited to values that the axis can follow without strong vibration. Special attention
should be paid to starting up.

When braking, the deceleration must be adjusted so that the axis can follow the set value ramp.

The creep velocity, creep distance and braking distance can be set to zero. The actual position must follow
the set position to avoid overshooting. If this is not the case, the pre-control must be reduced.

At this point, the axis is fully commissioned for positioning. If a pressure regulator, cam plate or gear coupling
is used in the application, these elements must also be put into operation.

386

Version: 1.8.3

TF5810

Support and Service

6 Support and Service

Beckhoff and their partners around the world offer comprehensive support and service, making available fast
and competent assistance with all questions related to Beckhoff products and system solutions.

Download finder

Our download finder contains all the files that we offer you for downloading. You will find application reports,
technical documentation, technical drawings, configuration files and much more.

The downloads are available in various formats.

Beckhoff's branch offices and representatives

Please contact your Beckhoff branch office or representative for local support and service on Beckhoff
products!

The addresses of Beckhoff's branch offices and representatives round the world can be found on our internet
page: www.beckhoff.com

You will also find further documentation for Beckhoff components there.

Beckhoff Support

Support offers you comprehensive technical assistance, helping you not only with the application of
individual Beckhoff products, but also with other, wide-ranging services:

• support

• design, programming and commissioning of complex automation systems

• and extensive training program for Beckhoff system components

Hotline:
e-mail:

+49 5246 963-157
support@beckhoff.com

Beckhoff Service

The Beckhoff Service Center supports you in all matters of after-sales service:

• on-site service

• repair service

• spare parts service

• hotline service

Hotline:
e-mail:

+49 5246 963-460
service@beckhoff.com

Beckhoff Headquarters

Beckhoff Automation GmbH & Co. KG

Huelshorstweg 20
33415 Verl
Germany

Phone:
e-mail:
web:

+49 5246 963-0
info@beckhoff.com

www.beckhoff.com

TF5810

Version: 1.8.3

387

Trademark statements

Beckhoff®,  ATRO®,  EtherCAT®,  EtherCAT  G®,  EtherCAT  G10®,  EtherCAT  P®,  MX-System®,  Safety  over  EtherCAT®,  TC/BSD®,  TwinCAT®,
TwinCAT/BSD®, TwinSAFE®, XFC®, XPlanar® and XTS® are registered and licensed trademarks of Beckhoff Automation GmbH.

Third-party trademark statements

EnDat is a trademark of Dr. Johannes Heidenhain GmbH.

Microsoft, Microsoft Azure, Microsoft Edge, PowerShell, Visual Studio, Windows and Xbox are trademarks of the Microsoft group of companies.

More Information:
www.beckhoff.com/tf5810

Beckhoff Automation GmbH & Co. KG
Hülshorstweg 20
33415 Verl
Germany
Phone: +49 5246 9630
info@beckhoff.com
www.beckhoff.com

