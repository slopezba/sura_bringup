# SURA bringup

## Control de vehículo y Alpha

El vehículo usa `/<robot>/controller/controller_manager`. Con `arms=single`
o `arms=dual`, Alpha usa `/<robot>/alpha/controller/controller_manager`.
Cada manager recibe exclusivamente sus componentes `ros2_control`; ambos
conservan el modelo cinemático completo. Los estados de articulaciones se
publican en `/<robot>/joint_states`, con componentes distintos por manager.
Los comandos y acciones de los brazos están bajo `/<robot>/alpha/controller`.

En el perfil dual, `task_priority_controller` se carga en el manager Alpha.
Su salida de vehículo usa `base_command_mode: topic` y publica `Twist` en
`/<robot>/controller/body_velocity/setpoint`; el modo `interfaces` sigue siendo
el valor por defecto para configuraciones con un único manager. Las rutas
públicas `/<robot>/controller/task_priority/...` se conservan con remapeos.
La GUI debe apuntar al nodo `/<robot>/alpha/controller/task_priority_controller`.

El servidor de trayectorias configura task priority cuando es necesario,
activa `body_velocity` en el manager del vehículo y después activa task priority
en Alpha. Si Alpha rechaza el cambio, restaura los controladores del vehículo.
Teleop observa el estado `~/active` de task priority y deja de publicar consignas
al vehículo mientras está activo. Al solicitar control manual, lo desactiva
antes de cambiar los controladores del vehículo.

Los perfiles Alpha configuran `navigator_timeout` y `setpoint_timeout` a 0.5 s
para poner a cero consignas cuando dejan de llegar datos. En otras
configuraciones, estos timeouts valen 0 (deshabilitados) por defecto.
