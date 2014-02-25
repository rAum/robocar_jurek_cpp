(defrule SensorMatch
(VelocitySensor1 ?s1)
(VelocitySensor2 ?s2)
(test (>= (abs (- ?s1 ?s2)) 0.001))
=>
(assert (VelocitySensor crashed)))

(defrule EmergencyControl
(VelocitySensor crashed)
=>
(assert (Use-Controller EmergencyController)))

(deffacts system
(VelocitySensor1 15)
(VelocitySensor2 0)
)
