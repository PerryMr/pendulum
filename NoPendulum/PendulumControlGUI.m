
function PendulumControlGUI()
    %PENDULUMCONTROLGUI_MODIFIED GUI para control directo del péndulo con condición de rango
    %   Versión modificada que solo activa control cuando θ ∈ [170°, 190°]
    %   Incluye indicadores visuales del estado de rango
    
    %% Inicialización de variables globales
    global gui_data;
    
    % Estructura para datos de la GUI
    gui_data = struct();
    gui_data.ctrl = []; % Objeto de comunicación
    gui_data.is_connected = false;
    gui_data.is_running = false;
    gui_data.current_controller = 0; % 0=PID, 1=LQR
    
    % Matrices del sistema LQR
    gui_data.A_aug = [0,    1,    0,    0,    0;
                     0,    0,    0,    0,    0;
                     0,    0,    0,    1,    0;
                     0,    0,    55.85, -0.374, 0;
                     1,    0,    0,    0,    0];
                     
    gui_data.B_aug = [0; 1.0; 0; 0.803; 0];
    gui_data.C_aug = eye(5);
    gui_data.D_aug = zeros(5,1);
    
    % Parámetros por defecto
    gui_data.pid_gains = [0.0, 0.0, 0.0]; % [Kp, Ki, Kd]
    gui_data.q_diag = [0, 0, 0, 0, 0]; % Diagonal de Q
    gui_data.r_weight = 0; % Peso R
    gui_data.lqr_gains = [0, 0, 0, 0, 0]; % K calculado
    
    % Datos para gráficas (6 observaciones)
    gui_data.max_points = 1000;
    gui_data.time_data = [];
    gui_data.sample_data = [];
    gui_data.theta_data = []; % Ángulo péndulo
    gui_data.phi_data = []; % Ángulo rotor
    gui_data.dtheta_data = []; % Velocidad péndulo
    gui_data.dphi_data = []; % Velocidad rotor
    gui_data.controller_data = []; % Tipo de controlador activo
    gui_data.error_data = []; % Error integral
    gui_data.status_data = []; % Estado del sistema
    
    % Variables de timing
    gui_data.start_timestamp = [];
    gui_data.last_timestamp = 0;
    gui_data.last_successful_read = tic;
    gui_data.sample_counter = 0;
    
    % Estados del sistema - MODIFICADOS para incluir nuevo estado
    gui_data.STATE_IDLE = 0;
    gui_data.STATE_CONTROL = 1;
    gui_data.STATE_STOPPED = 2;
    gui_data.STATE_WAITING_FOR_RANGE = 3; % NUEVO
    
    % Status codes - MODIFICADOS
    gui_data.STATUS_OK = 0;
    gui_data.STATUS_STP_MOVING = 1;
    gui_data.STATUS_CONTROL_ACTIVE = 2;
    gui_data.STATUS_UPRIGHT_ACHIEVED = 3;
    gui_data.STATUS_OUT_OF_RANGE = 4; % NUEVO
    
    % NUEVOS parámetros de rango
    gui_data.CONTROL_ACTIVATION_RANGE = 10.0; % ±10° desde 180°
    gui_data.CONTROL_DEACTIVATION_RANGE = 25.0; % ±25° desde 180°
    
    %% Crear interfaz gráfica centrada
    create_centered_gui();
    create_gui_layout();
    initialize_plots();
    
    % Timer para actualización
    gui_data.timer = timer('ExecutionMode', 'fixedRate', ...
                          'Period', 0.05, ... % 50ms
                          'TimerFcn', @update_display);
    
    fprintf('=== GUI CONTROL CON CONDICIÓN DE RANGO ===\n');
    fprintf('MODIFICACIÓN: Control solo activo en θ ∈ [170°, 190°]\n');
    fprintf('Funcionalidades:\n');
    fprintf('• Control PID/LQR con condición de rango\n');
    fprintf('• Monitoreo en tiempo real del estado de rango\n');
    fprintf('• Indicadores visuales mejorados\n');
    fprintf('• Guardado de datos\n');
    fprintf('Por favor conecte al puerto serial.\n');
end

function create_centered_gui()
    global gui_data;
    
    % Obtener dimensiones de pantalla
    screen_size = get(0, 'ScreenSize');
    screen_width = screen_size(3);
    screen_height = screen_size(4);
    
    % Dimensiones para control directo
    fig_width = 1200;
    fig_height = 790; % Ligeramente más alto para nuevos indicadores
    
    % Calcular posición centrada
    fig_x = (screen_width - fig_width) / 2;
    fig_y = (screen_height - fig_height) / 2;
    
    % Crear figura principal
    gui_data.fig = figure('Name', 'Control Péndulo Invertido - Control con Condición de Rango', ...
                         'Position', [fig_x, fig_y, fig_width, fig_height], ...
                         'Resize', 'on', ...
                         'CloseRequestFcn', @close_gui, ...
                         'MenuBar', 'none', ...
                         'ToolBar', 'none');
    
    figure(gui_data.fig);
end

function create_gui_layout()
    global gui_data;
    gui_data.y_offset = 100; % Ajustado para nuevos elementos
    
    %% Panel de parámetros (izquierda)
    gui_data.param_panel = uipanel('Parent', gui_data.fig, ...
                                  'Title', 'Parámetros y Control', ...
                                  'Position', [0.02, 0.02, 0.32, 0.96], ...
                                  'FontSize', 10, ...
                                  'FontWeight', 'bold');
    
    create_connection_controls();
    create_range_indicator(); % NUEVO
    create_controller_selection();
    create_pid_controls();
    create_lqr_controls();
    create_status_display();
    create_control_buttons();
    
    %% Panel de gráficas (derecha)
    gui_data.plot_panel = uipanel('Parent', gui_data.fig, ...
                                 'Title', 'Monitoreo Control', ...
                                 'Position', [0.36, 0.02, 0.62, 0.96], ...
                                 'FontSize', 10, ...
                                 'FontWeight', 'bold');
end

function create_connection_controls()
    global gui_data;
    
    % Conexión serial
    uicontrol('Parent', gui_data.param_panel, ...
             'Style', 'text', ...
             'String', 'Puerto Serial:', ...
             'Position', [10, 805-gui_data.y_offset, 100, 20], ...
             'HorizontalAlignment', 'left', ...
             'FontSize', 10);
         
    gui_data.port_edit = uicontrol('Parent', gui_data.param_panel, ...
                                  'Style', 'edit', ...
                                  'String', 'COM10', ...
                                  'Position', [120, 805-gui_data.y_offset, 80, 25], ...
                                  'FontSize', 9);
                              
    gui_data.connect_btn = uicontrol('Parent', gui_data.param_panel, ...
                                    'Style', 'pushbutton', ...
                                    'String', 'Conectar', ...
                                    'Position', [210, 805-gui_data.y_offset, 80, 25], ...
                                    'Callback', @connect_callback, ...
                                    'FontSize', 9, ...
                                    'BackgroundColor', [0, 0.8, 0]);
    
    gui_data.refresh_btn = uicontrol('Parent', gui_data.param_panel, ...
                                    'Style', 'pushbutton', ...
                                    'String', 'Actualizar', ...
                                    'Position', [300, 805-gui_data.y_offset, 70, 25], ...
                                    'Callback', @refresh_ports_callback, ...
                                    'FontSize', 9, ...
                                    'BackgroundColor', [0.9, 0.9, 0.9]);
end

% NUEVA función para crear indicador de rango
function create_range_indicator()
    global gui_data;
    
    % Indicador de rango de control
    uicontrol('Parent', gui_data.param_panel, ...
             'Style', 'text', ...
             'String', '═══ INDICADOR DE RANGO DE CONTROL ═══', ...
             'Position', [10, 775-gui_data.y_offset, 360, 20], ...
             'HorizontalAlignment', 'center', ...
             'FontSize', 10, ...
             'FontWeight', 'bold');
    
    % Estado del rango
    gui_data.range_status = uicontrol('Parent', gui_data.param_panel, ...
                                     'Style', 'text', ...
                                     'String', 'FUERA DE RANGO', ...
                                     'Position', [20, 745-gui_data.y_offset, 340, 25], ...
                                     'HorizontalAlignment', 'center', ...
                                     'FontSize', 12, ...
                                     'FontWeight', 'bold', ...
                                     'ForegroundColor', [0.8, 0, 0], ...
                                     'BackgroundColor', [1, 0.9, 0.9]);
    
    % Distancia desde 180°
    gui_data.distance_display = uicontrol('Parent', gui_data.param_panel, ...
                                         'Style', 'text', ...
                                         'String', 'Distancia desde 180°: -- °', ...
                                         'Position', [20, 715-gui_data.y_offset, 170, 20], ...
                                         'FontSize', 9);
    
    % Rango requerido
    gui_data.range_required = uicontrol('Parent', gui_data.param_panel, ...
                                       'Style', 'text', ...
                                       'String', 'Rango requerido: ±10.0°', ...
                                       'Position', [200, 715-gui_data.y_offset, 160, 20], ...
                                       'FontSize', 9, ...
                                       'ForegroundColor', [0, 0, 0.8]);
end

function create_controller_selection()
    global gui_data;
    
    % Selección de controlador
    uicontrol('Parent', gui_data.param_panel, ...
             'Style', 'text', ...
             'String', '═══ SELECCIÓN DE CONTROLADOR ═══', ...
             'Position', [10, 680-gui_data.y_offset, 360, 20], ...
             'HorizontalAlignment', 'center', ...
             'FontSize', 10, ...
             'FontWeight', 'bold');
    
    % Botones de selección
    gui_data.pid_btn = uicontrol('Parent', gui_data.param_panel, ...
                                'Style', 'pushbutton', ...
                                'String', 'PID', ...
                                'Position', [45, 640-gui_data.y_offset, 130, 35], ...
                                'FontSize', 12, ...
                                'FontWeight', 'bold', ...
                                'BackgroundColor', [0.2, 0.8, 0.2], ...
                                'ForegroundColor', 'white', ...
                                'Callback', @(~,~) select_controller(0));
                            
    gui_data.lqr_btn = uicontrol('Parent', gui_data.param_panel, ...
                                'Style', 'pushbutton', ...
                                'String', 'LQR', ...
                                'Position', [195, 640-gui_data.y_offset, 130, 35], ...
                                'FontSize', 12, ...
                                'FontWeight', 'bold', ...
                                'BackgroundColor', [0.7, 0.7, 0.7], ...
                                'ForegroundColor', 'black', ...
                                'Callback', @(~,~) select_controller(1));
end

function create_pid_controls()
    global gui_data;
    
    % Parámetros PID
    uicontrol('Parent', gui_data.param_panel, ...
             'Style', 'text', ...
             'String', '── PARÁMETROS PID ──', ...
             'Position', [10, 600-gui_data.y_offset, 200, 20], ...
             'HorizontalAlignment', 'center', ...
             'FontSize', 10, ...
             'FontWeight', 'bold');
    
    labels = {'Kp:', 'Ki:', 'Kd:'};
    defaults = {'0.0', '0.0', '0.0'};
    
    gui_data.pid_edits = [];
    
    for i = 1:3
        y_pos = 570 - (i-1)*30;
        
        uicontrol('Parent', gui_data.param_panel, ...
                 'Style', 'text', ...
                 'String', labels{i}, ...
                 'Position', [20, y_pos-gui_data.y_offset, 30, 20], ...
                 'HorizontalAlignment', 'left', ...
                 'FontSize', 9);
             
        gui_data.pid_edits(i) = uicontrol('Parent', gui_data.param_panel, ...
                                         'Style', 'edit', ...
                                         'String', defaults{i}, ...
                                         'Position', [60, y_pos-gui_data.y_offset, 100, 25], ...
                                         'FontSize', 9, ...
                                         'BackgroundColor', 'white');
    end
end

function create_lqr_controls()
    global gui_data;
    
    % Parámetros LQR
    uicontrol('Parent', gui_data.param_panel, ...
             'Style', 'text', ...
             'String', '── PARÁMETROS LQR ──', ...
             'Position', [10, 480-gui_data.y_offset, 200, 20], ...
             'HorizontalAlignment', 'center', ...
             'FontSize', 10, ...
             'FontWeight', 'bold');
    
    % Pesos Q
    q_labels = {'Q1:', 'Q2:', 'Q3:', 'Q4:', 'Q5:'};
    q_defaults = {'0', '0', '0', '0', '0'};
    
    gui_data.q_edits = [];
    
    % Primera fila: Q1, Q2, Q3
    for i = 1:3
        x_pos = 20 + (i-1)*85;
        
        uicontrol('Parent', gui_data.param_panel, ...
                 'Style', 'text', ...
                 'String', q_labels{i}, ...
                 'Position', [x_pos, 450-gui_data.y_offset, 25, 20], ...
                 'HorizontalAlignment', 'left', ...
                 'FontSize', 8);
             
        gui_data.q_edits(i) = uicontrol('Parent', gui_data.param_panel, ...
                                       'Style', 'edit', ...
                                       'String', q_defaults{i}, ...
                                       'Position', [x_pos+25, 450-gui_data.y_offset, 50, 25], ...
                                       'FontSize', 8);
    end
    
    % Segunda fila: Q4, Q5
    for i = 4:5
        x_pos = 20 + (i-4)*85;
        
        uicontrol('Parent', gui_data.param_panel, ...
                 'Style', 'text', ...
                 'String', q_labels{i}, ...
                 'Position', [x_pos, 420-gui_data.y_offset, 25, 20], ...
                 'HorizontalAlignment', 'left', ...
                 'FontSize', 8);
             
        gui_data.q_edits(i) = uicontrol('Parent', gui_data.param_panel, ...
                                       'Style', 'edit', ...
                                       'String', q_defaults{i}, ...
                                       'Position', [x_pos+25, 420-gui_data.y_offset, 50, 25], ...
                                       'FontSize', 8);
    end
    
    % Peso R y botón calcular
    uicontrol('Parent', gui_data.param_panel, ...
             'Style', 'text', ...
             'String', 'R:', ...
             'Position', [190, 420-gui_data.y_offset, 20, 20], ...
             'FontSize', 8);
         
    gui_data.r_edit = uicontrol('Parent', gui_data.param_panel, ...
                               'Style', 'edit', ...
                               'String', '0', ...
                               'Position', [215, 420-gui_data.y_offset, 50, 25], ...
                               'FontSize', 8);
    
    gui_data.calc_k_btn = uicontrol('Parent', gui_data.param_panel, ...
                                   'Style', 'pushbutton', ...
                                   'String', 'Calcular K', ...
                                   'Position', [280, 420-gui_data.y_offset, 80, 25], ...
                                   'Callback', @calculate_lqr_gains, ...
                                   'FontSize', 9, ...
                                   'BackgroundColor', [0.9, 0.9, 1]);
    
    % Mostrar ganancias K
    gui_data.k_display = uicontrol('Parent', gui_data.param_panel, ...
                                  'Style', 'text', ...
                                  'String', 'K = [0.000, 0.000, 0.000, 0.000, 0.000]', ...
                                  'Position', [20, 380-gui_data.y_offset, 340, 30], ...
                                  'HorizontalAlignment', 'center', ...
                                  'FontSize', 9);
end

function create_status_display()
    global gui_data;
    
    % Panel de información de estado
    uicontrol('Parent', gui_data.param_panel, ...
             'Style', 'text', ...
             'String', '═══ ESTADO DEL SISTEMA ═══', ...
             'Position', [10, 340-gui_data.y_offset, 360, 20], ...
             'HorizontalAlignment', 'center', ...
             'FontSize', 10, ...
             'FontWeight', 'bold');
    
    % Estado actual
    gui_data.system_status = uicontrol('Parent', gui_data.param_panel, ...
                                      'Style', 'text', ...
                                      'String', 'Estado: Sistema Inactivo', ...
                                      'Position', [20, 310-gui_data.y_offset, 340, 20], ...
                                      'HorizontalAlignment', 'center', ...
                                      'FontSize', 9, ...
                                      'FontWeight', 'bold', ...
                                      'BackgroundColor', [0.95, 0.95, 0.95]);
    
    % Tiempo transcurrido
    gui_data.time_display = uicontrol('Parent', gui_data.param_panel, ...
                                     'Style', 'text', ...
                                     'String', 'Tiempo: 0.0 s', ...
                                     'Position', [20, 285-gui_data.y_offset, 160, 20], ...
                                     'FontSize', 8);
    
    % Ángulo actual del péndulo
    gui_data.angle_display = uicontrol('Parent', gui_data.param_panel, ...
                                      'Style', 'text', ...
                                      'String', 'Ángulo θ: 0.0°', ...
                                      'Position', [200, 285-gui_data.y_offset, 160, 20], ...
                                      'FontSize', 8);
    
    % Error de control
    gui_data.error_display = uicontrol('Parent', gui_data.param_panel, ...
                                      'Style', 'text', ...
                                      'String', 'Error: 0.0', ...
                                      'Position', [20, 260-gui_data.y_offset, 160, 20], ...
                                      'FontSize', 8);
    
    % Estado de posición invertida
    gui_data.upright_display = uicontrol('Parent', gui_data.param_panel, ...
                                        'Style', 'text', ...
                                        'String', 'Invertido: NO', ...
                                        'Position', [200, 260-gui_data.y_offset, 160, 20], ...
                                        'FontSize', 8);
end

function create_control_buttons()
    global gui_data;
    
    % Estado de conexión
    gui_data.status_text = uicontrol('Parent', gui_data.param_panel, ...
                                    'Style', 'text', ...
                                    'String', 'Estado: Desconectado', ...
                                    'Position', [20, 220-gui_data.y_offset, 340, 25], ...
                                    'HorizontalAlignment', 'center', ...
                                    'FontSize', 11, ...
                                    'FontWeight', 'bold', ...
                                    'ForegroundColor', [0.8, 0, 0], ...
                                    'BackgroundColor', [0.95, 0.95, 0.95]);
    
    % Botones principales
    gui_data.start_btn = uicontrol('Parent', gui_data.param_panel, ...
                                  'Style', 'pushbutton', ...
                                  'String', '▶ INICIAR', ...
                                  'Position', [20, 170-gui_data.y_offset, 160, 40], ...
                                  'Callback', @start_control, ...
                                  'FontSize', 11, ...
                                  'FontWeight', 'bold', ...
                                  'BackgroundColor', [0, 0.8, 0], ...
                                  'ForegroundColor', 'white', ...
                                  'Enable', 'off');
    
    gui_data.stop_btn = uicontrol('Parent', gui_data.param_panel, ...
                                 'Style', 'pushbutton', ...
                                 'String', '⏹ PARAR', ...
                                 'Position', [200, 170-gui_data.y_offset, 160, 40], ...
                                 'Callback', @stop_control, ...
                                 'FontSize', 11, ...
                                 'FontWeight', 'bold', ...
                                 'BackgroundColor', [0.8, 0, 0], ...
                                 'ForegroundColor', 'white', ...
                                 'Enable', 'off');
    
    % Botones secundarios
    button_width = 85;
    spacing = 10;
    
    gui_data.home_btn = uicontrol('Parent', gui_data.param_panel, ...
                                 'Style', 'pushbutton', ...
                                 'String', 'Home', ...
                                 'Position', [20, 120-gui_data.y_offset, button_width, 30], ...
                                 'Callback', @set_home, ...
                                 'FontSize', 9, ...
                                 'Enable', 'off');
    
    gui_data.save_btn = uicontrol('Parent', gui_data.param_panel, ...
                                 'Style', 'pushbutton', ...
                                 'String', 'Guardar', ...
                                 'Position', [20 + button_width + spacing, 120-gui_data.y_offset, button_width, 30], ...
                                 'Callback', @save_data, ...
                                 'FontSize', 9, ...
                                 'BackgroundColor', [0, 0, 0.8], ...
                                 'ForegroundColor', 'white', ...
                                 'Enable', 'off');

    gui_data.step_mode_btn = uicontrol('Parent', gui_data.param_panel, ...
                                 'Style', 'pushbutton', ...
                                 'String', 'Modo 1/16', ...
                                 'Position', [20 + 2*(button_width + spacing), 120-gui_data.y_offset, button_width, 30], ...
                                 'Callback', @set_step_mode, ...
                                 'FontSize', 8, ...
                                 'Enable', 'off');
end

function initialize_plots()
    global gui_data;
    
    % Crear subplots 3x2 para monitoreo
    
    % 1. Estado del sistema - MODIFICADO para incluir nuevo estado
    gui_data.ax1 = subplot(3, 2, 1, 'Parent', gui_data.plot_panel);
    gui_data.line1 = plot(gui_data.ax1, NaN, NaN, 'k');
    title(gui_data.ax1, 'Estado del Sistema', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(gui_data.ax1, 'Muestra');
    ylabel(gui_data.ax1, 'Estado');
    grid(gui_data.ax1, 'on');
    ylim(gui_data.ax1, [-0.5, 4.5]); % Ampliado para nuevo estado
    set(gui_data.ax1, 'YTick', 0:4, 'YTickLabel', {'Idle', 'Control', 'Stop', 'Wait', 'OutRange'});
    xlim(gui_data.ax1, [0, 50]);
    
    % 2. Ángulo del péndulo con setpoint y rangos - MODIFICADO
    gui_data.ax2 = subplot(3, 2, 2, 'Parent', gui_data.plot_panel);
    hold(gui_data.ax2, 'on');
    
    % Zonas de rango
    gui_data.fill_activation = fill(gui_data.ax2, [0, 50, 50, 0], [170, 170, 190, 190], ...
                                   [0.8, 1, 0.8], 'EdgeColor', 'none', ...
                                   'FaceAlpha', 0.3, 'DisplayName', 'Zona Activación');
    gui_data.fill_maintenance = fill(gui_data.ax2, [0, 50, 50, 0], [155, 155, 205, 205], ...
                                    [1, 1, 0.8], 'EdgeColor', 'none', ...
                                    'FaceAlpha', 0.2, 'DisplayName', 'Zona Mantenimiento');
    
    gui_data.line2a = plot(gui_data.ax2, NaN, NaN, 'r--', 'LineWidth', 2, 'DisplayName', 'Setpoint (180°)');
    gui_data.line2b = plot(gui_data.ax2, NaN, NaN, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Ángulo Péndulo');
    title(gui_data.ax2, 'Ángulo del Péndulo con Zonas de Control', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(gui_data.ax2, 'Muestra');
    ylabel(gui_data.ax2, 'Grados');
    legend(gui_data.ax2, 'show', 'Location', 'best', 'FontSize', 7);
    grid(gui_data.ax2, 'on');
    ylim(gui_data.ax2, [0, 360]);
    xlim(gui_data.ax2, [0, 50]);
    hold(gui_data.ax2, 'off');
    
    % 3. Ángulo del rotor (stepper)
    gui_data.ax3 = subplot(3, 2, 3, 'Parent', gui_data.plot_panel);
    gui_data.line3 = plot(gui_data.ax3, NaN, NaN, 'k');
    title(gui_data.ax3, 'Ángulo Rotor (Stepper)', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(gui_data.ax3, 'Muestra');
    ylabel(gui_data.ax3, 'Grados');
    grid(gui_data.ax3, 'on');
    xlim(gui_data.ax3, [0, 50]);
    
    % 4. Velocidades angulares
    gui_data.ax4 = subplot(3, 2, 4, 'Parent', gui_data.plot_panel);
    hold(gui_data.ax4, 'on');
    gui_data.line4a = plot(gui_data.ax4, NaN, NaN, 'k', 'DisplayName', 'ω péndulo');
    gui_data.line4b = plot(gui_data.ax4, NaN, NaN, 'r', 'DisplayName', 'ω rotor');
    title(gui_data.ax4, 'Velocidades Angulares', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(gui_data.ax4, 'Muestra');
    ylabel(gui_data.ax4, 'Grados/s');
    legend(gui_data.ax4, 'show', 'Location', 'best', 'FontSize', 8);
    grid(gui_data.ax4, 'on');
    xlim(gui_data.ax4, [0, 50]);
    hold(gui_data.ax4, 'off');
    
    % 5. Controlador activo
    gui_data.ax5 = subplot(3, 2, 5, 'Parent', gui_data.plot_panel);
    gui_data.line5 = plot(gui_data.ax5, NaN, NaN, 'r-', 'LineWidth', 2);
    title(gui_data.ax5, 'Controlador Activo', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(gui_data.ax5, 'Muestra');
    ylabel(gui_data.ax5, 'Tipo');
    grid(gui_data.ax5, 'on');
    ylim(gui_data.ax5, [-0.5, 1.5]);
    set(gui_data.ax5, 'YTick', 0:1, 'YTickLabel', {'PID', 'LQR'});
    xlim(gui_data.ax5, [0, 50]);
    
    % 6. Distancia desde 180° - NUEVA gráfica
    gui_data.ax6 = subplot(3, 2, 6, 'Parent', gui_data.plot_panel);
    hold(gui_data.ax6, 'on');
    gui_data.line6a = plot(gui_data.ax6, NaN, NaN, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Distancia');
    gui_data.line6b = plot(gui_data.ax6, [0, 50], [10, 10], 'g--', 'LineWidth', 2, 'DisplayName', 'Límite Activación');
    gui_data.line6c = plot(gui_data.ax6, [0, 50], [25, 25], 'r--', 'LineWidth', 2, 'DisplayName', 'Límite Desactivación');
    title(gui_data.ax6, 'Distancia desde 180° y Límites de Control', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(gui_data.ax6, 'Muestra');
    ylabel(gui_data.ax6, 'Grados');
    legend(gui_data.ax6, 'show', 'Location', 'best', 'FontSize', 7);
    grid(gui_data.ax6, 'on');
    ylim(gui_data.ax6, [0, 50]);
    xlim(gui_data.ax6, [0, 50]);
    hold(gui_data.ax6, 'off');
    
    % Mejorar apariencia de todas las gráficas
    all_axes = [gui_data.ax1, gui_data.ax2, gui_data.ax3, gui_data.ax4, gui_data.ax5, gui_data.ax6];
    
    for i = 1:length(all_axes)
        set(all_axes(i), 'FontSize', 9);
        set(all_axes(i), 'GridAlpha', 0.3);
    end
    
    fprintf('Gráficas inicializadas para control con condición de rango\n');
end

%% Callbacks principales

function select_controller(controller_type)
    global gui_data;
    
    gui_data.current_controller = controller_type;
    
    % Actualizar apariencia
    if controller_type == 0
        set(gui_data.pid_btn, 'BackgroundColor', [0.2, 0.8, 0.2], 'ForegroundColor', 'white');
        set(gui_data.lqr_btn, 'BackgroundColor', [0.7, 0.7, 0.7], 'ForegroundColor', 'black');
        fprintf('Controlador PID seleccionado\n');
    else
        set(gui_data.pid_btn, 'BackgroundColor', [0.7, 0.7, 0.7], 'ForegroundColor', 'black');
        set(gui_data.lqr_btn, 'BackgroundColor', [0.2, 0.8, 0.2], 'ForegroundColor', 'white');
        fprintf('Controlador LQR seleccionado\n');
    end
    
    % Enviar al Arduino
    if gui_data.is_connected
        try
            controller_action = [controller_type, 0, 0, 0, 0, 0];
            gui_data.ctrl.step(gui_data.ctrl.CMD_SELECT_CONTROLLER, controller_action);
        catch ME
            fprintf('Error enviando selección: %s\n', ME.message);
        end
    end
end

function connect_callback(~, ~)
    global gui_data;
    
    if ~gui_data.is_connected
        % Intentar conectar
        port = get(gui_data.port_edit, 'String');
        
        if isempty(gui_data.ctrl)
            gui_data.ctrl = ControlComms(2.0, 0);
        end
        
        status = gui_data.ctrl.connect(port, 500000);
        
        if status == gui_data.ctrl.STATUS_OK
            pause(2); % Dar tiempo al Arduino para inicializar
            
            gui_data.is_connected = true;
            set(gui_data.connect_btn, 'String', 'Desconectar', 'BackgroundColor', [0.8, 0, 0]);
            set(gui_data.status_text, 'String', 'Estado: Conectado - Sistema Listo', 'ForegroundColor', [0, 0.6, 0]);
            set([gui_data.start_btn, gui_data.stop_btn, gui_data.home_btn, gui_data.save_btn, gui_data.step_mode_btn], 'Enable', 'on');
            
            % Configuración inicial
            try
                base_action = [0, 0, 0, 0, 0, 0];
                
                % Comando SET_STEP_MODE (modo 1/16)
                step_action = base_action;
                step_action(1) = 4;
                gui_data.ctrl.step(gui_data.ctrl.CMD_SET_STEP_MODE, step_action);
                pause(0.2);
                
                % Comando SET_HOME
                gui_data.ctrl.step(gui_data.ctrl.CMD_SET_HOME, base_action);
                pause(0.2);
                
                % Enviar selección de controlador
                controller_action = base_action;
                controller_action(1) = gui_data.current_controller;
                gui_data.ctrl.step(gui_data.ctrl.CMD_SELECT_CONTROLLER, controller_action);
                pause(0.2);
                
                fprintf('Arduino configurado para control con condición de rango\n');
                
            catch ME
                fprintf('Advertencia configurando Arduino: %s\n', ME.message);
            end
            
            fprintf('=== CONECTADO EXITOSAMENTE ===\n');
            fprintf('Puerto: %s | Baudrate: 500000\n', port);
            fprintf('Sistema listo para control con condición de rango\n');
            
        else
            msgbox('Error al conectar con Arduino. Verifique el puerto y que esté ejecutando el código correcto.', 'Error', 'error');
        end
    else
        % Desconectar
        disconnect_arduino();
    end
end

function refresh_ports_callback(~, ~)
    global gui_data;
    
    try
        temp_ctrl = ControlComms(1.0, 0);
        available_ports = temp_ctrl.getSerialList();
        
        if ~isempty(available_ports)
            fprintf('Puertos disponibles: %s\n', strjoin(available_ports, ', '));
            current_port = get(gui_data.port_edit, 'String');
            if isempty(current_port) || ~any(strcmp(available_ports, current_port))
                set(gui_data.port_edit, 'String', available_ports{1});
            end
        else
            fprintf('No se encontraron puertos seriales\n');
        end
    catch ME
        fprintf('Error obteniendo puertos: %s\n', ME.message);
    end
end

function calculate_lqr_gains(~, ~)
    global gui_data;
    
    try
        % Leer valores de la interfaz
        Q_diag = zeros(1, 5);
        for i = 1:5
            Q_diag(i) = str2double(get(gui_data.q_edits(i), 'String'));
        end
        R_val = str2double(get(gui_data.r_edit, 'String'));
        
        % Validar valores
        if any(isnan(Q_diag)) || isnan(R_val) || any(Q_diag <= 0) || R_val <= 0
            msgbox('Todos los valores deben ser números positivos', 'Error', 'error');
            return;
        end
        
        % Crear matrices
        Q = diag(Q_diag);
        R = R_val;
        
        % Calcular ganancias LQR
        gui_data.lqr_gains = lqr(gui_data.A_aug, gui_data.B_aug, Q, R);
        
        % Mostrar resultado
        k_str = sprintf('K = [%.3f, %.3f, %.3f, %.3f, %.3f]', gui_data.lqr_gains);
        set(gui_data.k_display, 'String', k_str);
        
        % Enviar al Arduino
        if gui_data.is_connected
            try
                lqr_action = [gui_data.lqr_gains, 0];
                gui_data.ctrl.step(gui_data.ctrl.CMD_SET_LQR_GAINS, lqr_action);
                fprintf('Ganancias LQR enviadas al Arduino\n');
            catch ME
                fprintf('Error enviando ganancias LQR: %s\n', ME.message);
            end
        end
        
        fprintf('Ganancias LQR calculadas: %s\n', k_str);
        
    catch ME
        msgbox(['Error calculando LQR: ' ME.message], 'Error', 'error');
    end
end

function start_control(~, ~)
    global gui_data;
    
    if ~gui_data.is_connected
        msgbox('Debe conectarse al Arduino primero', 'Error', 'error');
        return;
    end
    
    base_action = [0, 0, 0, 0, 0, 0];
    
    % Validar y enviar parámetros según controlador
    if gui_data.current_controller == 0 % PID
        try
            for i = 1:3
                gui_data.pid_gains(i) = str2double(get(gui_data.pid_edits(i), 'String'));
            end
            
            if any(isnan(gui_data.pid_gains))
                msgbox('Las ganancias PID deben ser números válidos', 'Error', 'error');
                return;
            end
            
            pid_action = [gui_data.pid_gains, 0, 0, 0];
            gui_data.ctrl.step(gui_data.ctrl.CMD_SET_PID_GAINS, pid_action);
            pause(0.1);
            
        catch
            msgbox('Error en los parámetros PID', 'Error', 'error');
            return;
        end
    else % LQR
        if all(gui_data.lqr_gains == 0)
            msgbox('Debe calcular las ganancias LQR primero', 'Error', 'error');
            return;
        end
    end
    
    % Limpiar datos y inicializar
    clear_data();
    
    % Enviar selección de controlador y comando de inicio
    controller_action = base_action;
    controller_action(1) = gui_data.current_controller;
    gui_data.ctrl.step(gui_data.ctrl.CMD_SELECT_CONTROLLER, controller_action);
    pause(0.1);
    
    gui_data.ctrl.step(gui_data.ctrl.CMD_START_CONTROL, base_action);
    
    % Actualizar estado
    gui_data.is_running = true;
    set(gui_data.start_btn, 'Enable', 'off');
    set(gui_data.stop_btn, 'Enable', 'on');
    
    % Iniciar timer
    start(gui_data.timer);
    
    controller_name = {'PID', 'LQR'};
    fprintf('=== CONTROL CON CONDICIÓN DE RANGO INICIADO ===\n');
    fprintf('Controlador: %s\n', controller_name{gui_data.current_controller + 1});
    fprintf('CONDICIÓN: Control solo activo cuando θ ∈ [170°, 190°]\n');
    fprintf('El sistema esperará automáticamente hasta que se cumpla la condición\n');
end

function stop_control(~, ~)
    global gui_data;
    
    if gui_data.is_connected
        base_action = [0, 0, 0, 0, 0, 0];
        gui_data.ctrl.step(gui_data.ctrl.CMD_STOP_CONTROL, base_action);
    end
    
    % Actualizar estado
    gui_data.is_running = false;
    set(gui_data.start_btn, 'Enable', 'on');
    set(gui_data.stop_btn, 'Enable', 'off');
    set(gui_data.system_status, 'String', 'Estado: Sistema Detenido', 'ForegroundColor', [0.8, 0.5, 0]);
    
    % Detener timer
    if isvalid(gui_data.timer)
        stop(gui_data.timer);
    end
    
    fprintf('=== CONTROL DETENIDO ===\n');
end

function set_home(~, ~)
    global gui_data;
    
    if gui_data.is_connected && ~gui_data.is_running
        base_action = [0, 0, 0, 0, 0, 0];
        gui_data.ctrl.step(gui_data.ctrl.CMD_SET_HOME, base_action);
        fprintf('Posición home establecida\n');
    end
end

function set_step_mode(~, ~)
    global gui_data;
    
    if gui_data.is_connected && ~gui_data.is_running
        base_action = [0, 0, 0, 0, 0, 0];
        base_action(1) = 4; % Modo 1/16
        gui_data.ctrl.step(gui_data.ctrl.CMD_SET_STEP_MODE, base_action);
        fprintf('Modo de pasos configurado: 1/16\n');
    end
end

function save_data(~, ~)
    global gui_data;
    
    if isempty(gui_data.sample_data)
        msgbox('No hay datos para guardar', 'Información', 'info');
        return;
    end
    
    try
        % Crear estructura de datos completa
        data_struct = struct();
        data_struct.sample = gui_data.sample_data;
        data_struct.time = gui_data.time_data;
        data_struct.theta = gui_data.theta_data;
        data_struct.phi = gui_data.phi_data;
        data_struct.dtheta = gui_data.dtheta_data;
        data_struct.dphi = gui_data.dphi_data;
        data_struct.controller = gui_data.controller_data;
        data_struct.error = gui_data.error_data;
        data_struct.status = gui_data.status_data;
        data_struct.current_controller = gui_data.current_controller;
        data_struct.control_activation_range = gui_data.CONTROL_ACTIVATION_RANGE;
        data_struct.control_deactivation_range = gui_data.CONTROL_DEACTIVATION_RANGE;
        
        if gui_data.current_controller == 0
            data_struct.pid_gains = gui_data.pid_gains;
        else
            data_struct.lqr_gains = gui_data.lqr_gains;
        end
        
        % Guardar archivo
        timestamp = datestr(now, 'yyyymmdd_HHMMSS');
        controller_name = {'PID', 'LQR'};
        filename = sprintf('pendulum_range_control_%s_%s.mat', controller_name{gui_data.current_controller + 1}, timestamp);
        
        save(filename, 'data_struct');
        
        % Guardar gráficas
        figure_copy = figure('Visible', 'off', 'Position', [0, 0, 1200, 800]);
        copyobj(allchild(gui_data.plot_panel), figure_copy);
        
        img_filename = sprintf('pendulum_range_control_%s_%s.png', controller_name{gui_data.current_controller + 1}, timestamp);
        saveas(figure_copy, img_filename);
        close(figure_copy);
        
        msgbox(sprintf('Datos guardados:\n%s\n%s', filename, img_filename), 'Guardado', 'info');
        
    catch ME
        msgbox(['Error guardando datos: ' ME.message], 'Error', 'error');
    end
end

% NUEVA función para calcular distancia desde 180°
function distance = calculate_distance_from_180(angle)
    distance = abs(angle - 180.0);
    if distance > 180.0
        distance = 360.0 - distance;
    end
end

% MODIFICADA función para actualizar información de estado
function update_status_info(obs, status, current_time)
    global gui_data;
    
    theta = obs(1);
    error_integral = obs(6);
    
    % Calcular distancia desde 180°
    distance_180 = calculate_distance_from_180(theta);
    
    % Actualizar displays básicos
    set(gui_data.time_display, 'String', sprintf('Tiempo: %.1f s', current_time));
    set(gui_data.angle_display, 'String', sprintf('θ: %.1f°', theta));
    set(gui_data.error_display, 'String', sprintf('Error: %.3f', error_integral));
    
    % NUEVO: Actualizar indicador de distancia
    set(gui_data.distance_display, 'String', sprintf('Distancia desde 180°: %.1f°', distance_180));
    
    % NUEVO: Actualizar indicador de rango
    is_in_activation_range = distance_180 <= gui_data.CONTROL_ACTIVATION_RANGE;
    is_in_maintenance_range = distance_180 <= gui_data.CONTROL_DEACTIVATION_RANGE;
    
    if is_in_activation_range
        set(gui_data.range_status, 'String', '✓ EN RANGO DE ACTIVACIÓN', ...
            'ForegroundColor', [0, 0.8, 0], 'BackgroundColor', [0.9, 1, 0.9]);
    elseif is_in_maintenance_range
        set(gui_data.range_status, 'String', '~ EN RANGO DE MANTENIMIENTO', ...
            'ForegroundColor', [0.8, 0.6, 0], 'BackgroundColor', [1, 1, 0.9]);
    else
        set(gui_data.range_status, 'String', '✗ FUERA DE RANGO', ...
            'ForegroundColor', [0.8, 0, 0], 'BackgroundColor', [1, 0.9, 0.9]);
    end
    
    % Verificar si está invertido (umbral estricto)
    is_upright = distance_180 <= 15.0;
    
    if is_upright
        set(gui_data.upright_display, 'String', 'Invertido: SÍ', 'ForegroundColor', [0, 0.8, 0]);
    else
        set(gui_data.upright_display, 'String', 'Invertido: NO', 'ForegroundColor', [0.8, 0, 0]);
    end
    
    % MODIFICADO: Actualizar según estado con nuevos estados
    switch status
        case gui_data.STATUS_OK
            set(gui_data.system_status, 'String', 'Estado: Sistema Listo', 'ForegroundColor', [0, 0.6, 0]);
            
        case gui_data.STATUS_STP_MOVING
            set(gui_data.system_status, 'String', 'Estado: Motor en Movimiento', 'ForegroundColor', [0.6, 0.6, 0]);
            
        case gui_data.STATUS_CONTROL_ACTIVE
            controller_name = {'PID', 'LQR'};
            set(gui_data.system_status, 'String', sprintf('Estado: Control %s Activo', controller_name{gui_data.current_controller + 1}), 'ForegroundColor', [0, 0, 0.8]);
            
        case gui_data.STATUS_UPRIGHT_ACHIEVED
            set(gui_data.system_status, 'String', '🎉 ¡PÉNDULO INVERTIDO CONTROLADO!', 'ForegroundColor', [0, 0.8, 0]);
            
        case gui_data.STATUS_OUT_OF_RANGE % NUEVO
            set(gui_data.system_status, 'String', '⚠️ ESPERANDO RANGO DE ACTIVACIÓN', 'ForegroundColor', [0.8, 0.5, 0]);
    end
end

function update_display(~, ~)
    global gui_data;
    
    if ~gui_data.is_connected || isempty(gui_data.ctrl)
        return;
    end
    
    try
        base_action = [0, 0, 0, 0, 0, 0];
        
        % Usar comando que no interfiera con el control
        resp = gui_data.ctrl.step(gui_data.ctrl.CMD_MOVE_BY, base_action);
        
        if ~isempty(resp) && isstruct(resp) && isfield(resp, 'observation') && isfield(resp, 'timestamp')
            timestamp = resp.timestamp;
            obs = resp.observation;
            status = resp.status;
            
            % Validación de datos
            if ~isempty(obs) && length(obs) >= 6 && all(isfinite(obs(1:6)))
                % Calcular tiempo relativo
                if isempty(gui_data.time_data)
                    gui_data.start_timestamp = timestamp;
                    current_time = 0;
                else
                    current_time = (timestamp - gui_data.start_timestamp) / 1000.0;
                end
                
                gui_data.sample_counter = gui_data.sample_counter + 1;
                
                % Agregar datos
                gui_data.sample_data(end+1) = gui_data.sample_counter;
                gui_data.time_data(end+1) = current_time;
                gui_data.theta_data(end+1) = obs(1);  % Ángulo péndulo
                gui_data.phi_data(end+1) = obs(2);    % Ángulo rotor
                gui_data.dtheta_data(end+1) = obs(3); % Velocidad péndulo
                gui_data.dphi_data(end+1) = obs(4);   % Velocidad rotor
                gui_data.controller_data(end+1) = obs(5); % Controlador activo
                gui_data.error_data(end+1) = obs(6);  % Error integral
                gui_data.status_data(end+1) = status;
                
                % Actualizar información de estado
                update_status_info(obs, status, current_time);
                
                % Limitar puntos para rendimiento
                if length(gui_data.sample_data) > gui_data.max_points
                    fields = {'sample_data', 'time_data', 'theta_data', 'phi_data', ...
                             'dtheta_data', 'dphi_data', 'controller_data', 'error_data', 'status_data'};
                    for i = 1:length(fields)
                        gui_data.(fields{i})(1) = [];
                    end
                end
                
                % Actualizar gráficas
                update_plots();
                
                % Debug cada 50 muestras
                if mod(gui_data.sample_counter, 2) == 0
                    distance = calculate_distance_from_180(obs(1));
                    fprintf('Muestra %d: θ=%.1f°, dist_180=%.1f°, Status=%d\n', ...
                        gui_data.sample_counter, obs(1), distance, status);
                end
            end
        end
        
    catch ME
        % Solo reportar errores críticos
        if toc(gui_data.last_successful_read) > 3
            fprintf('Error comunicación: %s\n', ME.message);
            gui_data.last_successful_read = tic;
        end
    end
end

% MODIFICADA función para actualizar gráficas
function update_plots()
    global gui_data;
    
    if isempty(gui_data.sample_data) || length(gui_data.sample_data) < 2
        return;
    end
    
    try
        n_points = length(gui_data.sample_data);
        samples = gui_data.sample_data;
        
        % Determinar ventana de visualización (últimas 200 muestras)
        if n_points > 200
            idx_start = n_points - 199;
            idx_end = n_points;
            x_data = samples(idx_start:idx_end);
        else
            idx_start = 1;
            idx_end = n_points;
            x_data = samples;
        end
        
        if isempty(x_data) || length(x_data) < 2
            return;
        end
        
        % 1. Estado del sistema - MODIFICADO para nuevos estados
        if length(gui_data.status_data) >= idx_end
            y_data = gui_data.status_data(idx_start:idx_end);
            if ~isempty(y_data) && length(y_data) == length(x_data)
                set(gui_data.line1, 'XData', x_data, 'YData', y_data);
            end
        end
        
        % 2. Ángulo del péndulo con setpoint y zonas - MODIFICADO
        if length(gui_data.theta_data) >= idx_end
            theta_data = gui_data.theta_data(idx_start:idx_end);
            if ~isempty(theta_data) && length(theta_data) == length(x_data)
                % Actualizar zonas de rango
                x_range = [min(x_data)-1, max(x_data)+1, max(x_data)+1, min(x_data)-1];
                set(gui_data.fill_activation, 'XData', x_range);
                set(gui_data.fill_maintenance, 'XData', x_range);
                
                setpoint_data = 180 * ones(size(theta_data));
                set(gui_data.line2a, 'XData', x_data, 'YData', setpoint_data);
                set(gui_data.line2b, 'XData', x_data, 'YData', theta_data);
                xlim(gui_data.ax2, [min(x_data)-1, max(x_data)+1]);
                
                % Ajustar límites Y dinámicamente
                theta_min = min(theta_data);
                theta_max = max(theta_data);
                if theta_max > theta_min
                    margin = (theta_max - theta_min) * 0.1;
                    ylim(gui_data.ax2, [max(0, theta_min - margin), min(360, theta_max + margin)]);
                end
            end
        end
        
        % 3. Ángulo del rotor
        if length(gui_data.phi_data) >= idx_end
            phi_data = gui_data.phi_data(idx_start:idx_end);
            if ~isempty(phi_data) && length(phi_data) == length(x_data)
                set(gui_data.line3, 'XData', x_data, 'YData', phi_data);
                xlim(gui_data.ax3, [min(x_data)-1, max(x_data)+1]);
                
                % Ajustar límites Y dinámicamente
                phi_min = min(phi_data);
                phi_max = max(phi_data);
                if phi_max > phi_min
                    phi_range = phi_max - phi_min;
                    margin = max(phi_range * 0.1, 10);
                    ylim(gui_data.ax3, [phi_min - margin, phi_max + margin]);
                end
            end
        end
        
        % 4. Velocidades angulares
        if length(gui_data.dtheta_data) >= idx_end && length(gui_data.dphi_data) >= idx_end
            dtheta_data = gui_data.dtheta_data(idx_start:idx_end);
            dphi_data = gui_data.dphi_data(idx_start:idx_end);
            if ~isempty(dtheta_data) && ~isempty(dphi_data) && ...
               length(dtheta_data) == length(x_data) && length(dphi_data) == length(x_data)
                set(gui_data.line4a, 'XData', x_data, 'YData', dtheta_data);
                set(gui_data.line4b, 'XData', x_data, 'YData', dphi_data);
                xlim(gui_data.ax4, [min(x_data)-1, max(x_data)+1]);
                
                % Ajustar límites Y para velocidades
                all_vel_data = [dtheta_data(:); dphi_data(:)];
                vel_min = min(all_vel_data);
                vel_max = max(all_vel_data);
                if vel_max > vel_min
                    vel_range = vel_max - vel_min;
                    margin = max(vel_range * 0.1, 50);
                    ylim(gui_data.ax4, [vel_min - margin, vel_max + margin]);
                end
            end
        end
        
        % 5. Controlador activo
        if length(gui_data.controller_data) >= idx_end
            controller_data = gui_data.controller_data(idx_start:idx_end);
            if ~isempty(controller_data) && length(controller_data) == length(x_data)
                set(gui_data.line5, 'XData', x_data, 'YData', controller_data);
                xlim(gui_data.ax5, [min(x_data)-1, max(x_data)+1]);
            end
        end
        
        % 6. Distancia desde 180° - NUEVA gráfica
        if length(gui_data.theta_data) >= idx_end
            theta_data = gui_data.theta_data(idx_start:idx_end);
            if ~isempty(theta_data) && length(theta_data) == length(x_data)
                % Calcular distancias
                distance_data = arrayfun(@calculate_distance_from_180, theta_data);
                
                set(gui_data.line6a, 'XData', x_data, 'YData', distance_data);
                
                % Actualizar líneas de límites
                x_range_limits = [min(x_data)-1, max(x_data)+1];
                set(gui_data.line6b, 'XData', x_range_limits);
                set(gui_data.line6c, 'XData', x_range_limits);
                
                xlim(gui_data.ax6, [min(x_data)-1, max(x_data)+1]);
                
                % Ajustar límites Y para distancia
                dist_max = max(distance_data);
                ylim(gui_data.ax6, [0, max(35, dist_max + 5)]);
            end
        end
        
        % Forzar actualización
        drawnow;
        
    catch ME
        fprintf('Error actualizando gráficas: %s\n', ME.message);
    end
end

function clear_data()
    global gui_data;
    
    % Limpiar todos los arrays de datos
    fields = {'time_data', 'sample_data', 'theta_data', 'phi_data', 'dtheta_data', ...
             'dphi_data', 'controller_data', 'error_data', 'status_data'};
    
    for i = 1:length(fields)
        gui_data.(fields{i}) = [];
    end
    
    gui_data.start_timestamp = [];
    gui_data.last_timestamp = 0;
    gui_data.sample_counter = 0;
end

function disconnect_arduino()
    global gui_data;
    
    if gui_data.is_running
        stop_control();
    end
    
    if gui_data.is_connected && ~isempty(gui_data.ctrl)
        try
            gui_data.ctrl.close();
        catch ME
            fprintf('Error cerrando conexión: %s\n', ME.message);
        end
        gui_data.is_connected = false;
    end
    
    % Actualizar interfaz
    set(gui_data.connect_btn, 'String', 'Conectar', 'BackgroundColor', [0, 0.8, 0]);
    set(gui_data.status_text, 'String', 'Estado: Desconectado', 'ForegroundColor', [0.8, 0, 0]);
    set([gui_data.start_btn, gui_data.stop_btn, gui_data.home_btn, gui_data.save_btn, gui_data.step_mode_btn], 'Enable', 'off');
    
    fprintf('=== DESCONECTADO DEL ARDUINO ===\n');
end

function close_gui(~, ~)
    global gui_data;
    
    % Detener timer
    if isfield(gui_data, 'timer') && isvalid(gui_data.timer)
        try
            stop(gui_data.timer);
            delete(gui_data.timer);
        catch
            % Ignorar errores
        end
    end
    
    % Desconectar Arduino
    disconnect_arduino();
    
    % Cerrar figura
    if isfield(gui_data, 'fig') && isvalid(gui_data.fig)
        delete(gui_data.fig);
    end
    
    fprintf('=== GUI CERRADA ===\n');
end
