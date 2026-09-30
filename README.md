# drone-ros

Software de a bordo y de misión del dron de reparto urbano de última milla, sobre ROS 2 Jazzy y PX4 v1.17.

> Proyecto de aprendizaje personal. Sigue un proceso inspirado en ARP4754A/ARP4761A, **sin certificación**.

- **Nivel de control:** CC1 (control completo: baseline y cambios con informe de problema), según el plan de configuración (DOC-09 §5).
- **Cada cambio empieza por un issue** (plantilla «Problema o cambio») y entra por pull request con CI en verde y autorrevisión.

## Contenido

| Ruta | Qué es |
| --- | --- |
| `drone_core/` | Librería C++ sin ROS: geodesia, geometría, configuración por ciudad, validación de misiones. Incluye `drone_validate` |
| `drone_interfaces/` | Mensajes, servicios y acción del ICD |
| `drone_mission/` | `MissionStateMachine` y los nodos `config_manager` y `mission_manager` |
| `drone_payload/` | Suelta de carga: `PayloadStateMachine` (C++ puro) y el nodo `payload_manager` (acción `DropPayload`, confirmación del piloto, sensor de carga) |
| `drone_s1_demo/` | Demostración del hito S1: despegue, estacionario y aterrizaje |
| `ops/` | Configuración por ciudad (YAML + GeoJSON): toulouse, donostia |
| `missions/` | Misiones de ejemplo |

`ops/` y `missions/` son datos que cambian el comportamiento del sistema: `config_manager` verifica su hash.
Los cambios en ellos son cambios de configuración CC1.

## Cómo se relaciona con los otros repos

- [`drone-sim`](https://github.com/XabMS/drone-sim): instala y fija las versiones de todo (`drone.repos`) y clona este repo en `ros2_ws/src/drone-ros`.
- [`drone-px4`](https://github.com/XabMS/drone-px4): fork de PX4 con `drop_guard`.
- [`drone-docs`](https://github.com/XabMS/drone-docs): documentación y requisitos.

`px4_msgs` (rama `release/1.17`) no se incluye: se compila aparte y se fija en `drone.repos` de `drone-sim`. Como esa rama no trae
`DropGuardStatus.msg` (lo añade el fork `drone-px4`), hay que copiarlo del `msg/` del fork dentro de `px4_msgs` antes de compilar
(`setup_native.sh` lo hace; el CI de este repo lo descarga del tag del fork).

## Compilar y probar

Lo normal es usar el entorno de `drone-sim` (`setup_native.sh` deja este repo clonado en `ros2_ws/src/drone-ros`).
Para compilar por libre, hace falta ROS 2 Jazzy, `px4_msgs` en el workspace, `libyaml-cpp-dev` y `nlohmann-json3-dev`:

```bash
colcon build --symlink-install
colcon test --packages-select drone_core drone_mission drone_payload && colcon test-result --verbose   # 101 tests: 64 de drone_core y drone_mission + 37 de drone_payload
```

Validar una ciudad y una misión sin arrancar nada:

```bash
ros2 run drone_core drone_validate ops toulouse missions tls_demo_01
```

`drone_s1_demo` no tiene tests, por eso se excluye de `colcon test`.

Los paquetes se compilan con `-Werror` (`drone_core`, `drone_mission` y `drone_payload`): no debe haber avisos.

## Flujo de cambios

1. Abrir un issue con la plantilla «Problema o cambio» (descripción, análisis de impacto, decisión, verificación de cierre).
2. Una rama de trabajo por cambio y una pull request contra `main`.
3. `main` está protegida: exige PR y el check `build-test` en verde; no hay push directo ni historial con merges.
4. Autorrevisión con el checklist de la PR (idealmente 48 h después de escribir el cambio).

## Licencia

Apache-2.0 (ver [`LICENSE`](LICENSE)).
