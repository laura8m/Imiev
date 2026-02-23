# Historial de Cambios - Proyecto iMiev (Cronológico)

Este diario registra la evolución de la configuración del vehículo para facilitar el seguimiento de las pruebas y retroceder si es necesario.

---

## 📅 Log de Versiones / Iteraciones

### 🔄 Configuración Inicial y Renombrado
- **Renombrado del Paquete**: Cambio masivo de `mutt_robot` a `imiev` en todo el workspace (config, launch, setup).
- **Corrección URDF Base**: Ajuste de geometría (chasis elevado a 0.82m) y alineación de ruedas.
- **TF Sensores**: Configuración de `lidar_link`, `camera_link`, `gps_link` e `imu_link`.

### 🔄 Ajuste de Cinemática Ackermann (V1)
- **Radio de Giro**: Establecido en **3.65m** (Wheelbase 2.5m / Steer 0.6 rad).
- **Namespace Fix**: Cambio de `constraints:` a `AckermannConstraints:` en el YAML (Nav2 Jazzy).
- **Estabilidad Lidar**: `transform_tolerance: 3.0` para evitar mensajes descartados por lag.

### 🔄 Optimización de Entorno y BT
- **Fricción del Suelo**: Configuración de `mu: 0.8` y `fdir1: 0 0 0` en las ruedas para asfalto realista.
- **Behavior Trees**: Eliminada dependencia de `Spin` (incompatible con Ackermann) y ajustado `BackUp` a 1.5m.
- **Frecuencias Costmap**: Subido Local Map a 5Hz y Global Map a 2Hz.

### 🔄 Iteración: Giro Agresivo
- **Muestreo MPPI**: Doblado ruido angular (`wz_std: 0.4`).
- **Control Directo**: `temperature: 0.1` para decisiones más tajantes.
- **Restricción Estricta**: `ConstraintCritic` a 10.0 para obligar al cumplimiento del radio.
- **Twirling Fix**: `TwirlingCritic: 0.0` para permitir giros fluidos.

### 🔄 Iteración: Rendimiento y Planning
- **Resolución "Slow Motion"**: Ajustado `inflation_radius: 3.3m` (mínimo técnico para evitar el error de "circumscribed radius").
- **Batch Processing**: Aumentado `batch_size: 2000` y `time_steps: 56` para una "inteligencia" superior de búsqueda.
- **model_dt**: Sincronizado a 0.05 para coherencia con 20Hz.

### 🔄 Iteración: Agilidad vs Búfer Local
- **Separación Local/Global**: 
    - Global: 5.0m (Seguridad y planning rápido).
    - Local: 3.3m (Maniobrabilidad en corto).
- **Critic Rebalance**: `PathAlign: 14.0`, `Constraint: 4.0` (Basado en el archivo de referencia `nav2_params.yaml`).

### 🔄 Iteración Actual: Sintonización "Safe & Smooth"
- **Sintonización Suave y Segura**:
    - `temperature: 0.15` y ruidos (`0.2`/`0.4`) para eliminar el "jitter" (movimientos cortos).
    - `CostCritic: 30.0` y `PathAlignCritic: 10.0` para priorizar la seguridad sobre la ruta.
    - Local Inflation: `4.0m` con `scaling: 1.0` para un margen de seguridad amplio y progresivo.
- **Velocidad Dinámica**:
    - Aumentado `vx_max` a 1.0m/s y `wz_max` a 1.0 rad/s para dar más "inercia" y autoridad al giro.
    - **Corrección Bottleneck**: Detectado y eliminado límite de `0.26 m/s` en el `velocity_smoother`. Ahora el coche puede alcanzar su velocidad objetivo de 1.0m/s.
- **Prioridad de Marcha Adelante**:
    - `reverse_penalty: 5.0` (Global) y `PreferForwardCritic: 5.0` (Local) para forzar giros de frente en lugar de usar revesa tan fácilmente.
