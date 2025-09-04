%% GUI para control directo del péndulo invertido
% Solo control PID/LQR directo del péndulo

function PendulumControlGUI()
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
    
    % Variables para Step Response y Frequency Response
    gui_data.step_response_active = false;
    gui_data.step_start_time = 0;
    gui_data.step_start_angle = 0;
    gui_data.step_response_data = [];
    gui_data.step_time_data = [];
    
    % Variables de timing
    gui_data.start_timestamp = [];
    gui_data.last_timestamp = 0;
    gui_data.last_successful_read = tic;
    gui_data.sample_counter = 0;
    
    % Estados del sistema
    gui_data.STATE_IDLE = 0;
    gui_data.STATE_CONTROL = 1;
    gui_data.STATE_STOPPED = 2;
    
    % Status codes
    gui_data.STATUS_OK = 0;
    gui_data.STATUS_STP_MOVING = 1;
    gui_data.STATUS_CONTROL_ACTIVE = 2;
    gui_data.STATUS_UPRIGHT_ACHIEVED = 3;
    
    %% Crear interfaz gráfica centrada
    create_centered_gui();
    create_gui_layout();
    initialize_plots();
    
    % Timer para actualización
    gui_data.timer = timer('ExecutionMode', 'fixedRate', ...
                          'Period', 0.02, ... % 50ms
                          'TimerFcn', @update_display);
    
    fprintf('=== GUI CONTROL DIRECTO PÉNDULO ===\n');
    fprintf('Funcionalidades:\n');
    fprintf('• Control PID/LQR directo\n');
    fprintf('• Monitoreo en tiempo real\n');
    fprintf('• Step Response Analysis\n');
    fprintf('• Frequency Response (Bode)\n');
    fprintf('• Guardado de datos\n');
    fprintf('Por favor conecte al puerto serial.\n');
end

%% Centralización de la ventana
function create_centered_gui()
    global gui_data;
    
    % Obtener dimensiones de pantalla
    screen_size = get(0, 'ScreenSize');
    screen_width = screen_size(3);
    screen_height = screen_size(4);
    
    % Dimensiones extendidas para nuevas gráficas
    fig_width = 1500;
    fig_height = 790;
    
    % Calcular posición centrada
    fig_x = (screen_width - fig_width) / 2;
    fig_y = (screen_height - fig_height) / 2;
    
    % Crear figura principal
    gui_data.fig = figure('Name', 'Control Péndulo Invertido', ...
                         'Position', [fig_x, fig_y, fig_width, fig_height], ...
                         'Resize', 'on', ...
                         'CloseRequestFcn', @close_gui, ...
                         'MenuBar', 'none', ...
                         'ToolBar', 'none');
    
    figure(gui_data.fig);
end

%% Separación de la ventana en tres recuadros
function create_gui_layout()
    global gui_data;
    gui_data.y_offset = 100;
    
    %% Panel de parámetros (izquierda)
    gui_data.param_panel = uipanel('Parent', gui_data.fig, ...
                                  'Title', 'Parámetros y Control', ...
                                  'Position', [0.02, 0.02, 0.28, 0.96], ...
                                  'FontSize', 10, ...
                                  'FontWeight', 'bold');
    
    create_connection_controls(); % Puerto serial
    create_controller_selection(); % Selección del controlador
    create_pid_controls(); % Controlador PID
    create_lqr_controls(); % Controlador LQR
    create_analysis_controls();  % Gráficas Step y Bode
    create_status_display(); % Actualización de las gráficas
    create_control_buttons(); % Botones
    
    %% Panel de gráficas principales (derecha-arriba)
    gui_data.plot_panel = uipanel('Parent', gui_data.fig, ...
                                 'Title', 'Monitoreo Control Directo', ...
                                 'Position', [0.32, 0.42, 0.66, 0.56], ...
                                 'FontSize', 10, ...
                                 'FontWeight', 'bold');
    
    %% Panel de análisis (derecha-abajo)
    gui_data.analysis_panel = uipanel('Parent', gui_data.fig, ...
                                     'Title', 'Análisis del Sistema', ...
                                     'Position', [0.32, 0.02, 0.66, 0.4], ...
                                     'FontSize', 10, ...
                                     'FontWeight', 'bold');
end

%% Puerto Seria-conexión 
function create_connection_controls()
    global gui_data;
    
    % Conexión serial
    uicontrol('Parent', gui_data.param_panel, ...
             'Style', 'text', ...
             'String', 'Puerto Serial:', ...
             'Position', [10, 800-gui_data.y_offset, 100, 20], ...
             'HorizontalAlignment', 'left', ...
             'FontSize', 10);
         
    gui_data.port_edit = uicontrol('Parent', gui_data.param_panel, ...
                                  'Style', 'edit', ...
                                  'String', 'COM10', ...
                                  'Position', [120, 800-gui_data.y_offset, 80, 25], ...
                                  'FontSize', 9);
                              
    gui_data.connect_btn = uicontrol('Parent', gui_data.param_panel, ...
                                    'Style', 'pushbutton', ...
                                    'String', 'Conectar', ...
                                    'Position', [210, 800-gui_data.y_offset, 80, 25], ...
                                    'Callback', @connect_callback, ...
                                    'FontSize', 9, ...
                                    'BackgroundColor', [0, 0.8, 0]);
    
    gui_data.refresh_btn = uicontrol('Parent', gui_data.param_panel, ...
                                    'Style', 'pushbutton', ...
                                    'String', 'Actualizar', ...
                                    'Position', [300, 800-gui_data.y_offset, 70, 25], ...
                                    'Callback', @refresh_ports_callback, ...
                                    'FontSize', 9, ...
                                    'BackgroundColor', [0.9, 0.9, 0.9]);
end

%% Tipos de controladores
function create_controller_selection()
    global gui_data;
    
    % Selección de controlador
    uicontrol('Parent', gui_data.param_panel, ...
             'Style', 'text', ...
             'String', '|═══ SELECCIÓN DE CONTROLADOR ═══|', ...
             'Position', [10, 760-gui_data.y_offset, 360, 20], ...
             'HorizontalAlignment', 'center', ...
             'FontSize', 10, ...
             'FontWeight', 'bold');
    
    % Botones de selección
    gui_data.pid_btn = uicontrol('Parent', gui_data.param_panel, ...
                                'Style', 'pushbutton', ...
                                'String', 'PID', ...
                                'Position', [45, 720-gui_data.y_offset, 130, 35], ...
                                'FontSize', 12, ...
                                'FontWeight', 'bold', ...
                                'BackgroundColor', [0.2, 0.8, 0.2], ...
                                'ForegroundColor', 'white', ...
                                'Callback', @(~,~) select_controller(0));
                            
    gui_data.lqr_btn = uicontrol('Parent', gui_data.param_panel, ...
                                'Style', 'pushbutton', ...
                                'String', 'LQR', ...
                                'Position', [195, 720-gui_data.y_offset, 130, 35], ...
                                'FontSize', 12, ...
                                'FontWeight', 'bold', ...
                                'BackgroundColor', [0.7, 0.7, 0.7], ...
                                'ForegroundColor', 'black', ...
                                'Callback', @(~,~) select_controller(1));
end

%% Espacios para ingresar los parámetro para el PID
function create_pid_controls()
    global gui_data;
    
    % Parámetros PID
    uicontrol('Parent', gui_data.param_panel, ...
             'Style', 'text', ...
             'String', '|── PARÁMETROS PID ──| ', ...
             'Position', [10, 680-gui_data.y_offset, 200, 20], ...
             'HorizontalAlignment', 'center', ...
             'FontSize', 10, ...
             'FontWeight', 'bold');
    
    labels = {'Kp:', 'Ki:', 'Kd:'};
    defaults = {'0.0', '0.0', '0.0'};
    
    gui_data.pid_edits = [];
    
    for i = 1:3
        y_pos = 650 - (i-1)*30;
        
        uicontrol('Parent', gui_data.param_panel, ...
                 'Style', 'text', ...
                 'String', labels{i}, ...
                 'Position', [20, y_pos-gui_data.y_offset, 30, 20], ...
                 'HorizontalAlignment', 'left', ...
                 'FontSize', 9);
             
        gui_data.pid_edits(i) = uicontrol('Parent', gui_data.param_panel, ...
                                         'Style', 'edit', ...
                                         'String', defaults{i}, ...
                                         'Position', [50, y_pos-gui_data.y_offset, 100, 25], ...
                                         'FontSize', 9, ...
                                         'BackgroundColor', 'white');
    end
end

%% Espacios para ingresar los parámetros para el LQR
function create_lqr_controls()
    global gui_data;
    
    % Parámetros LQR
    uicontrol('Parent', gui_data.param_panel, ...
             'Style', 'text', ...
             'String', '|── PARÁMETROS LQR ──|', ...
             'Position', [205, 680-gui_data.y_offset, 200, 20], ...
             'HorizontalAlignment', 'center', ...
             'FontSize', 10, ...
             'FontWeight', 'bold');
    
    % Pesos Q
    q_labels = {'Q1:', 'Q2:', 'Q3:', 'Q4:', 'Q5:'};
    q_defaults = {'0', '0', '0', '0', '0'};
    
    gui_data.q_edits = [];
    
    % Primera fila: Q1, Q2, Q3
    for i = 1:3
        y_pos = 650 - (i-1)*30;
        
        uicontrol('Parent', gui_data.param_panel, ...
                 'Style', 'text', ...
                 'String', q_labels{i}, ...
                 'Position', [210, y_pos-gui_data.y_offset, 25, 20], ...
                 'HorizontalAlignment', 'left', ...
                 'FontSize', 8);
             
        gui_data.q_edits(i) = uicontrol('Parent', gui_data.param_panel, ...
                                       'Style', 'edit', ...
                                       'String', q_defaults{i}, ...
                                       'Position', [240, y_pos-gui_data.y_offset, 50, 25], ...
                                       'FontSize', 8);
    end
    
    % Segunda fila: Q4, Q5
    for i = 4:5
        y_pos = 740 - (i-1)*30;
        
        uicontrol('Parent', gui_data.param_panel, ...
                 'Style', 'text', ...
                 'String', q_labels{i}, ...
                 'Position', [310, y_pos-gui_data.y_offset, 25, 20], ...
                 'HorizontalAlignment', 'left', ...
                 'FontSize', 8);
             
        gui_data.q_edits(i) = uicontrol('Parent', gui_data.param_panel, ...
                                       'Style', 'edit', ...
                                       'String', q_defaults{i}, ...
                                       'Position', [340, y_pos-gui_data.y_offset, 50, 25], ...
                                       'FontSize', 8);
    end
    
    % Peso R y botón calcular
    uicontrol('Parent', gui_data.param_panel, ...
             'Style', 'text', ...
             'String', 'R:', ...
             'Position', [310, 590-gui_data.y_offset, 20, 20], ...
             'FontSize', 8);
         
    gui_data.r_edit = uicontrol('Parent', gui_data.param_panel, ...
                               'Style', 'edit', ...
                               'String', '0', ...
                               'Position', [340, 590-gui_data.y_offset, 50, 25], ...
                               'FontSize', 8);
    
    gui_data.calc_k_btn = uicontrol('Parent', gui_data.param_panel, ...
                                   'Style', 'pushbutton', ...
                                   'String', 'Calcular K', ...
                                   'Position', [60, 550-gui_data.y_offset, 80, 25], ...
                                   'Callback', @calculate_lqr_gains, ...
                                   'FontSize', 9, ...
                                   'BackgroundColor', [0.9, 0.9, 1]);
    
    % Mostrar ganancias K
    gui_data.k_display = uicontrol('Parent', gui_data.param_panel, ...
                                  'Style', 'text', ...
                                  'String', 'K = [0.000, 0.000, 0.000, 0.000, 0.000]', ...
                                  'Position', [165, 545-gui_data.y_offset, 200, 25], ...
                                  'HorizontalAlignment', 'center', ...
                                  'FontSize', 9);
end

%% Botones para generar gráficas Step y Bode
function create_analysis_controls()
    global gui_data;
    
    % NUEVA SECCIÓN: Controles para análisis
    uicontrol('Parent', gui_data.param_panel, ...
             'Style', 'text', ...
             'String', '|═══ ANÁLISIS DEL SISTEMA ═══|', ...
             'Position', [10, 505-gui_data.y_offset, 360, 20], ...
             'HorizontalAlignment', 'center', ...
             'FontSize', 10, ...
             'FontWeight', 'bold');
    
    % Botón para Step Response
    gui_data.step_response_btn = uicontrol('Parent', gui_data.param_panel, ...
                                          'Style', 'pushbutton', ...
                                          'String', 'Step Response', ...
                                          'Position', [20, 475-gui_data.y_offset, 120, 25], ...
                                          'Callback', @generate_step_response, ...
                                          'FontSize', 9, ...
                                          'BackgroundColor', [0.9, 0.9, 1], ...
                                          'ForegroundColor', 'black', ...
                                          'Enable', 'on');
    
    % Botón para Frequency Response
    gui_data.freq_response_btn = uicontrol('Parent', gui_data.param_panel, ...
                                          'Style', 'pushbutton', ...
                                          'String', 'Bode Plot', ...
                                          'Position', [160, 475-gui_data.y_offset, 120, 25], ...
                                          'Callback', @generate_bode_plot, ...
                                          'FontSize', 9, ...
                                          'BackgroundColor', [0.9, 0.9, 1], ...
                                          'ForegroundColor', 'black', ...
                                          'Enable', 'on');
    
    % Botón para limpiar análisis
    gui_data.clear_analysis_btn = uicontrol('Parent', gui_data.param_panel, ...
                                           'Style', 'pushbutton', ...
                                           'String', 'Limpiar Análisis', ...
                                           'Position', [290, 475-gui_data.y_offset, 90, 25], ...
                                           'Callback', @clear_analysis, ...
                                           'FontSize', 8, ...
                                           'BackgroundColor', [0.6, 0.6, 0.6], ...
                                           'Enable', 'off');
end

%% Visualización algunos parámetros del sistema
function create_status_display()
    global gui_data;
    
    % Panel de información de estado
    uicontrol('Parent', gui_data.param_panel, ...
             'Style', 'text', ...
             'String', '|═══ ESTADO DEL SISTEMA ═══|', ...
             'Position', [10, 435-gui_data.y_offset, 360, 20], ...
             'HorizontalAlignment', 'center', ...
             'FontSize', 10, ...
             'FontWeight', 'bold');
    
    % Estado actual
    gui_data.system_status = uicontrol('Parent', gui_data.param_panel, ...
                                      'Style', 'text', ...
                                      'String', 'Estado: Sistema Inactivo', ...
                                      'Position', [20, 410-gui_data.y_offset, 340, 20], ...
                                      'HorizontalAlignment', 'center', ...
                                      'FontSize', 9, ...
                                      'FontWeight', 'bold', ...
                                      'BackgroundColor', [0.95, 0.95, 0.95]);
    
    % Tiempo transcurrido
    gui_data.time_display = uicontrol('Parent', gui_data.param_panel, ...
                                     'Style', 'text', ...
                                     'String', 'Tiempo: 0.0 s', ...
                                     'Position', [20, 390-gui_data.y_offset, 160, 20], ...
                                     'FontSize', 8);
    
    % Ángulo actual del péndulo
    gui_data.angle_display = uicontrol('Parent', gui_data.param_panel, ...
                                      'Style', 'text', ...
                                      'String', 'Ángulo θ: 0.0°', ...
                                      'Position', [200, 390-gui_data.y_offset, 160, 20], ...
                                      'FontSize', 8);
    
    % Error de control
    gui_data.error_display = uicontrol('Parent', gui_data.param_panel, ...
                                      'Style', 'text', ...
                                      'String', 'Error: 0.0', ...
                                      'Position', [20, 365-gui_data.y_offset, 160, 20], ...
                                      'FontSize', 8);
    
    % Estado de posición invertida
    gui_data.upright_display = uicontrol('Parent', gui_data.param_panel, ...
                                        'Style', 'text', ...
                                        'String', 'Invertido: NO', ...
                                        'Position', [200, 365-gui_data.y_offset, 160, 20], ...
                                        'FontSize', 8);
end

%% Botones para iniciar con la simulación y utilidad
function create_control_buttons()
    global gui_data;
    
    % Estado de conexión
    gui_data.status_text = uicontrol('Parent', gui_data.param_panel, ...
                                    'Style', 'text', ...
                                    'String', 'Estado: Desconectado', ...
                                    'Position', [20, 330-gui_data.y_offset, 340, 25], ...
                                    'HorizontalAlignment', 'center', ...
                                    'FontSize', 11, ...
                                    'FontWeight', 'bold', ...
                                    'ForegroundColor', [0.8, 0, 0], ...
                                    'BackgroundColor', [0.95, 0.95, 0.95]);
    
    % Botones principales
    gui_data.start_btn = uicontrol('Parent', gui_data.param_panel, ...
                                  'Style', 'pushbutton', ...
                                  'String', '▶ INICIAR', ...
                                  'Position', [20, 290-gui_data.y_offset, 160, 40], ...
                                  'Callback', @start_control, ...
                                  'FontSize', 11, ...
                                  'FontWeight', 'bold', ...
                                  'BackgroundColor', [0, 0.8, 0], ...
                                  'ForegroundColor', 'white', ...
                                  'Enable', 'off');
    
    gui_data.stop_btn = uicontrol('Parent', gui_data.param_panel, ...
                                 'Style', 'pushbutton', ...
                                 'String', '⏹ PARAR', ...
                                 'Position', [200, 290-gui_data.y_offset, 160, 40], ...
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
                                 'Position', [20, 250-gui_data.y_offset, button_width, 30], ...
                                 'Callback', @set_home, ...
                                 'FontSize', 9, ...
                                 'Enable', 'off');
    
    gui_data.save_btn = uicontrol('Parent', gui_data.param_panel, ...
                                 'Style', 'pushbutton', ...
                                 'String', 'Guardar', ...
                                 'Position', [20 + button_width + spacing, 250-gui_data.y_offset, button_width, 30], ...
                                 'Callback', @save_data, ...
                                 'FontSize', 9, ...
                                 'BackgroundColor', [0, 0, 0.8], ...
                                 'ForegroundColor', 'white', ...
                                 'Enable', 'off');

    gui_data.step_mode_btn = uicontrol('Parent', gui_data.param_panel, ...
                                 'Style', 'pushbutton', ...
                                 'String', 'Modo 1/16', ...
                                 'Position', [20 + 2*(button_width + spacing), 250-gui_data.y_offset, button_width, 30], ...
                                 'Callback', @set_step_mode, ...
                                 'FontSize', 8, ...
                                 'Enable', 'off');
                            
end

%% Creación de las gráficas para la visualización en tiempo real (derecha-arriba)
function initialize_plots()
    global gui_data;
    
    %% Crear axes manualmente para el panel de monitoreo (2x2)
    % Definir posiciones para subplots 2x2 en el panel superior
    subplot_width = 0.42;
    subplot_height = 0.36;
    x_positions = [0.05, 0.53];
    y_positions = [0.59, 0.1];
    
    % 1. Ángulo del péndulo con setpoint
    gui_data.ax1 = axes('Parent', gui_data.plot_panel, ...
                       'Position', [x_positions(1), y_positions(1), subplot_width, subplot_height]);
    hold(gui_data.ax1, 'on');
    gui_data.line1a = plot(gui_data.ax1, NaN, NaN, 'r--', 'LineWidth', 2, 'DisplayName', 'Setpoint (180°)');
    gui_data.line1b = plot(gui_data.ax1, NaN, NaN, 'k-', 'DisplayName', 'Ángulo Péndulo');
    title(gui_data.ax1, 'Ángulo del Péndulo', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(gui_data.ax1, 'Muestra');
    ylabel(gui_data.ax1, 'Grados');
    legend(gui_data.ax1, 'show', 'Location', 'best', 'FontSize', 8);
    grid(gui_data.ax1, 'on');
    ylim(gui_data.ax1, [0, 360]);
    xlim(gui_data.ax1, [0, 50]);
    hold(gui_data.ax1, 'off');
    
    % 2. Ángulo del rotor (stepper)
    gui_data.ax2 = axes('Parent', gui_data.plot_panel, ...
                       'Position', [x_positions(2), y_positions(1), subplot_width, subplot_height]);
    gui_data.line2 = plot(gui_data.ax2, NaN, NaN, 'k');
    title(gui_data.ax2, 'Ángulo Rotor (Stepper)', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(gui_data.ax2, 'Muestra');
    ylabel(gui_data.ax2, 'Grados');
    grid(gui_data.ax2, 'on');
    xlim(gui_data.ax2, [0, 50]);
    
    % 3. Velocidades angulares
    gui_data.ax3 = axes('Parent', gui_data.plot_panel, ...
                       'Position', [x_positions(1), y_positions(2), subplot_width, subplot_height]);
    hold(gui_data.ax3, 'on');
    gui_data.line3a = plot(gui_data.ax3, NaN, NaN, 'k', 'DisplayName', 'ω péndulo');
    gui_data.line3b = plot(gui_data.ax3, NaN, NaN, 'r', 'DisplayName', 'ω rotor');
    title(gui_data.ax3, 'Velocidades Angulares', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(gui_data.ax3, 'Muestra');
    ylabel(gui_data.ax3, 'Grados/s');
    legend(gui_data.ax3, 'show', 'Location', 'best', 'FontSize', 8);
    grid(gui_data.ax3, 'on');
    xlim(gui_data.ax3, [0, 50]);
    hold(gui_data.ax3, 'off');
    
    % 4. Controlador activo
    gui_data.ax4 = axes('Parent', gui_data.plot_panel, ...
                       'Position', [x_positions(2), y_positions(2), subplot_width, subplot_height]);
    gui_data.line4 = plot(gui_data.ax4, NaN, NaN, 'r-', 'LineWidth', 2);
    title(gui_data.ax4, 'Controlador Activo', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(gui_data.ax4, 'Muestra');
    ylabel(gui_data.ax4, 'Tipo');
    grid(gui_data.ax4, 'on');
    ylim(gui_data.ax4, [-0.5, 1.5]);
    set(gui_data.ax4, 'YTick', 0:1, 'YTickLabel', {'PID', 'LQR'});
    xlim(gui_data.ax4, [0, 50]);
    
    %% Crear axes manualmente para el panel de análisis
    
    % Definir posiciones para el panel de análisis
    analysis_width = 0.44;
    analysis_height = 0.75;
    bode_width = 0.4;
    bode_height = 0.35;
    
    % 5. Step Response (lado izquierdo)
    gui_data.ax6 = axes('Parent', gui_data.analysis_panel, ...
                           'Position', [0.05, 0.15, analysis_width, analysis_height]);
    gui_data.step_line_response = plot(gui_data.ax6, NaN, NaN, 'k');
    title(gui_data.ax6, 'Step Response', 'FontSize', 12);
    xlabel(gui_data.ax6, 'Tiempo (s)');
    ylabel(gui_data.ax6, 'Ángulo Péndulo (°)');
    grid(gui_data.ax6, 'on');
    xlim(gui_data.ax6, [0, 10]);
    ylim(gui_data.ax6, [0, 360]);
    
    % 6. Bode Plot - Magnitud (arriba derecha)
    gui_data.ax_bode_mag = axes('Parent', gui_data.analysis_panel, ...
                               'Position', [0.55, 0.6, bode_width, bode_height]);
    gui_data.line_bode_mag = semilogx(gui_data.ax_bode_mag, NaN, NaN, 'k');
    title(gui_data.ax_bode_mag, 'Magnitude (dB)', 'FontSize', 10);
    ylabel(gui_data.ax_bode_mag, 'Magnitude (dB)');
    grid(gui_data.ax_bode_mag, 'on');
    
    % 7. Bode Plot - Fase (abajo derecha)
    gui_data.ax_bode_phase = axes('Parent', gui_data.analysis_panel, ...
                                 'Position', [0.55, 0.13, bode_width, bode_height]);
    gui_data.line_bode_phase = semilogx(gui_data.ax_bode_phase, NaN, NaN, 'k');
    title(gui_data.ax_bode_phase, 'Phase (degrees)', 'FontSize', 10);
    xlabel(gui_data.ax_bode_phase, 'Frequency (rad/s)');
    ylabel(gui_data.ax_bode_phase, 'Phase (degrees)');
    grid(gui_data.ax_bode_phase, 'on');
    
    % Mejorar apariencia de todas las gráficas
    all_axes = [gui_data.ax1, gui_data.ax2, gui_data.ax3, gui_data.ax4, ...
                gui_data.ax6, gui_data.ax_bode_mag, gui_data.ax_bode_phase];
    
    for i = 1:length(all_axes)
        if isvalid(all_axes(i))
            set(all_axes(i), 'FontSize', 9);
            try
                set(all_axes(i), 'GridAlpha', 0.3);
            catch
            end
        end
    end
    fprintf('Gráficas inicializadas para control directo con análisis\n');
end

%% FUNCIONES PARA LAS GRÁFICAS STEP Y BODE PLOT
function generate_step_response(~, ~)
    global gui_data;
    
    fprintf('=== GENERANDO STEP RESPONSE TEÓRICO ===\n');
    
    try
        if gui_data.current_controller == 0 % PID
            fprintf('Calculando Step Response para controlador PID\n');
            generate_pid_step_response();
        else % LQR
            fprintf('Calculando Step Response para controlador LQR\n');
            generate_lqr_step_response();
        end
        
        fprintf('Step Response generado exitosamente\n');
        
    catch ME
        fprintf('Error generando Step Response: %s\n', ME.message);
        msgbox(['Error generando Step Response: ' ME.message], 'Error', 'error');
    end
end

%% Step Response Gráfica - PID
function generate_pid_step_response()
    global gui_data;
    
    % Leer ganancias PID actuales
    Kp = str2double(get(gui_data.pid_edits(1), 'String'));
    Ki = str2double(get(gui_data.pid_edits(2), 'String'));
    Kd = str2double(get(gui_data.pid_edits(3), 'String'));
    
    fprintf('Ganancias PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n', Kp, Ki, Kd);
    
    s=tf('s');
    l=0.258;
    r=0.141;
    g=9.806; 
    sigma=g/(l*(2*pi*1.19)^2); %sigma=2/3+Irotor
%     wr=sqrt(r/(l*(sigma)));
    wr2=r/(l*(sigma));
    wg=sqrt(g/(l*(sigma)));
    wg2=g/(l*(sigma));
    Q=20;
    Gps=wr2*s^2/(s^2+(wg/Q)*s-wg2); %Planta del pendulo
    Grs=1/s^2; %Planta del rotor
    gui_data.Grrs=Grs*Gps; %Producto de la planta en lazo abierto
    
    % Verificar polos de la planta (debe ser inestable)
    poles_plant = pole(gui_data.Grrs);
    fprintf('Polos de la planta: ');
    for i = 1:length(poles_plant)
        if imag(poles_plant(i)) == 0
            fprintf('%.2f ', real(poles_plant(i)));
        else
            fprintf('%.2f±%.2fi ', real(poles_plant(i)), abs(imag(poles_plant(i))));
        end
    end
    fprintf('\n');
    if any(real(poles_plant) > 0)
        fprintf('✓ Planta inestable (correcto para péndulo invertido)\n');
    end
    
    % CONTROLADOR PID
    % C(s) = Kp+Ki/s+Kd·s = (Kd·s²+Kp·s+Ki)/s    
    % Numerador del PID: Kd·s² + Kp·s + Ki
    num_pid = [Kd, Kp, Ki];
    % Denominador del PID: s
    den_pid = [1, 0];
    % Crear controlador PID
    C_pid = tf(num_pid, den_pid);
    
    % SISTEMA EN LAZO CERRADO
    % Diagrama: Ref_θ → [C(s)] → φ → [G(s)] → θ → [realimentación] ↑
    
    try
        % 1. Función de transferencia directa (forward path)
        % L(s) = C(s) × G(s) = PID × Planta
        Gls = C_pid*gui_data.Grrs;
        % 2. Sistema en lazo cerrado con realimentación unitaria negativa
        % T(s) = L(s) / (1 + L(s))
        Gcls = feedback(Gls, 1);        
    catch ME
        fprintf('Error formando lazo cerrado: %s\n', ME.message);
        return;
    end
    
    % Generar respuesta al escalón
    t_final = 1; % 10 segundos
    dt = 0.01;    % 10 ms
    t = 0:dt:t_final;
    
    % Escalón de 180° (posición vertical)
    step_input = 180;
    
    % Calcular respuesta
    [y, t_out] = step(Gcls*step_input, t);
    
    % Verificar estabilidad del lazo cerrado
    Gcls_poles = pole(Gcls);
    fprintf('Polos del sistema en lazo cerrado: ');
    stable = true;
    for i = 1:length(Gcls_poles)
        if imag(Gcls_poles(i)) == 0
            fprintf('%.2f ', real(Gcls_poles(i)));
        else
            fprintf('%.2f±%.2fi ', real(Gcls_poles(i)), abs(imag(Gcls_poles(i))));
        end
        if real(Gcls_poles(i)) >= 0
            stable = false;
        end
    end
    fprintf('\n');
    
    if stable
        fprintf('✓Sistema en lazo cerrado ESTABLE\n');
    else
        fprintf('⚠ADVERTENCIA: Sistema INESTABLE - Ajustar ganancias PID\n');
    end
    
    % Actualizar gráfica
    set(gui_data.step_line_response, 'XData', t_out, 'YData', y);
    
    % Ajustar límites
    xlim(gui_data.ax6, [0, t_final]);
    y_min = min(y);
    y_max = max(y);
    margin = (y_max - y_min);
    
    % Si el rango es muy pequeño, usar un rango mínimo
    if margin < 0.1
        y_center = (y_max + y_min) / 2;
        y_min_plot = y_center - 0.1;
        y_max_plot = y_center + 0.1;
    else
        % Agregar margen del 10% arriba y abajo
        margin_percent = 0.15;  % 15% de margen
        y_margin = margin * margin_percent;
        y_min_plot = y_min - y_margin;
        y_max_plot = y_max + y_margin;
    end
    
    % Asegurar que incluya el cero y la referencia (1 para escalón unitario)
    y_min_plot = min(y_min_plot, -0.1);  % Incluir un poco debajo de cero
    y_max_plot = max(y_max_plot, 1.2);   % Incluir un poco arriba de 1
    
    ylim(gui_data.ax6, [y_min_plot, y_max_plot]);
    
    try % Información del sistema
        info = stepinfo(Gcls*step_input); 
        fprintf('Información del Step Response PID:\n');
        if isfield(info, 'RiseTime') && ~isempty(info.RiseTime)
            fprintf('  Tiempo de subida: %.2f s\n', info.RiseTime);
        end
        if isfield(info, 'SettlingTime') && ~isempty(info.SettlingTime)
            fprintf('  Tiempo de asentamiento: %.2f s\n', info.SettlingTime);
        end
        if isfield(info, 'Overshoot') && ~isempty(info.Overshoot)
            fprintf('  Sobreimpulso: %.2f%%\n', info.Overshoot);
        end
        if isfield(info, 'SteadyStateValue') && ~isempty(info.SteadyStateValue)
            fprintf('  Valor final: %.2f°\n', info.SteadyStateValue);
        end
        if isfield(info, 'Peak') && isfinite(info.Peak)
            fprintf('  Valor pico: %.2f°\n', info.Peak);
        end
        if isfield(info, 'PeakTime') && isfinite(info.PeakTime)
            fprintf('  Tiempo del pico: %.3f s\n', info.PeakTime);
        end
        
        % Error en estado estacionario

        step_amplitude = 180;
        final_value = y(end);
        steady_state_error = step_amplitude - final_value;
        error_percent = abs(steady_state_error) / step_amplitude * 100;

        fprintf('Valor final: %.2f°\n', final_value);
        fprintf('Error estado estac.: %.2f°\n', steady_state_error);
        fprintf('Error porcentual: %.2f%%\n', error_percent);
    
    catch ME
        fprintf('Error en análisis: %s\n', ME.message);
    end    
end

%% Step response Gráfica - LQR
function generate_lqr_step_response()
    global gui_data;
    
    % Verificar que las ganancias LQR están calculadas
    if all(gui_data.lqr_gains == 0)
        msgbox('Debe calcular las ganancias LQR primero usando el botón "Calcular K"', 'Error', 'error');
        return;
    end
    
    fprintf('Ganancias LQR: [%.2f, %.2f, %.2f, %.2f, %.2f]\n', gui_data.lqr_gains);
    
    % Sistema aumentado (incluye integrador)
    A = gui_data.A_aug;
    B = gui_data.B_aug;
    K = gui_data.lqr_gains;
    
    % Sistema en lazo cerrado
    A_cl = A - B * K;
    
    % Vector de entrada para referencia (escalón)
    % La referencia entra en el integrador (último estado)
    B_ref = [0; 0; 0; 0; 1];
    
    % Salida: ángulo del péndulo (tercer estado)
    C_cl = [0, 0, 1, 0, 0];
    D_cl = 0;
    
    % Crear sistema en lazo cerrado
    sys_cl = ss(A_cl, B_ref, C_cl, D_cl);
    
    % Verificar estabilidad
    poles = pole(sys_cl);
    fprintf('Polos del sistema LQR: ');
    stable = true;
    for i = 1:length(poles)
        if imag(poles(i)) == 0
            fprintf('%.2f ', real(poles(i)));
        else
            fprintf('%.2f±%.2fi ', real(poles(i)), abs(imag(poles(i))));
        end
        if real(poles(i)) >= 0
            stable = false;
        end
    end
    fprintf('\n');
    
    if stable
        fprintf('✓ Sistema LQR ESTABLE\n');
    else
        fprintf('⚠ Sistema LQR INESTABLE\n');
    end
    
    % Generar respuesta al escalón
    t_final = 1; % 10 segundos
    dt = 0.01;    % 10 ms
    t = 0:dt:t_final;
    
    % Escalón de 180° (posición vertical) + offset para estado inicial
    % El sistema LQR está diseñado para regular alrededor del punto de equilibrio
    % Aplicamos un escalón desde 0° hasta 180°
    step_input = 180;
    
    % Calcular respuesta
    [y, t_out] = step(sys_cl*step_input, t);
    
    % Actualizar gráfica
    set(gui_data.step_line_response, 'XData', t_out, 'YData', y);
    
    % Ajustar límites
    xlim(gui_data.ax6, [0, t_final]);
    y_min = min(y);
    y_max = max(y);
    margin = (y_max - y_min);
    
    % Si el rango es muy pequeño, usar un rango mínimo
    if margin < 0.1
        y_center = (y_max + y_min) / 2;
        y_min_plot = y_center - 0.1;
        y_max_plot = y_center + 0.1;
    else
        % Agregar margen del 10% arriba y abajo
        margin_percent = 0.15;  % 15% de margen
        y_margin = margin * margin_percent;
        y_min_plot = y_min - y_margin;
        y_max_plot = y_max + y_margin;
    end
    
    % Asegurar que incluya el cero y la referencia (1 para escalón unitario)
    y_min_plot = min(y_min_plot, -0.1);  % Incluir un poco debajo de cero
    y_max_plot = max(y_max_plot, 1.2);   % Incluir un poco arriba de 1
    
    ylim(gui_data.ax6, [y_min_plot, y_max_plot]);

    % Información del sistema
    try
        info = stepinfo(sys_cl*step_input); % Escalón de 180°
        fprintf('Información del Step Response LQR:\n');
        if isfield(info, 'RiseTime') && ~isempty(info.RiseTime) && isfinite(info.RiseTime)
            fprintf('  Tiempo de subida: %.2f s\n', info.RiseTime);
        end
        if isfield(info, 'SettlingTime') && ~isempty(info.SettlingTime) && isfinite(info.SettlingTime)
            fprintf('  Tiempo de asentamiento: %.2f s\n', info.SettlingTime);
        end
        if isfield(info, 'Overshoot') && ~isempty(info.Overshoot) && isfinite(info.Overshoot)
            fprintf('  Sobreimpulso: %.2f%%\n', info.Overshoot);
        end
        if isfield(info, 'SteadyStateValue') && ~isempty(info.SteadyStateValue) && isfinite(info.SteadyStateValue)
            fprintf('  Valor final: %.2f°\n', info.SteadyStateValue);
        end
        
        final_value = y(end);
        steady_state_error = step_amplitude - final_value;
        error_percent = abs(steady_state_error) / step_amplitude * 100;
        
        fprintf('║Valor final: %15.2f° ║\n', final_value);
        fprintf('║Error estado estac.: %9.2f° ║\n', steady_state_error);
        fprintf('║Error porcentual: %10.2f%% ║\n', error_percent);
        
    catch ME
        fprintf('No se pudo calcular stepinfo: %s\n', ME.message);
    end
end

%% Bode Plot - Versión modificada para Lazo Cerrado
function generate_bode_plot(~, ~)
    global gui_data;
    
    try
        % Crear función de transferencia basada en el controlador actual
        if gui_data.current_controller == 0  % PID
            % Función de transferencia PID: G(s) = Kp + Ki/s + Kd*s
            Kp = gui_data.pid_gains(1);
            Ki = gui_data.pid_gains(2);
            Kd = gui_data.pid_gains(3);
            
            % PID: G(s) = (Kd*s^2 + Kp*s + Ki) / s
            num_pid = [Kd, Kp, Ki];
            den_pid = [1, 0];
            G_controller = tf(num_pid, den_pid);
            
        else  % LQR
            if all(gui_data.lqr_gains == 0)
                msgbox('Debe calcular las ganancias LQR primero', 'Error', 'error');
                return;
            end
            
            % Crear sistema en lazo cerrado con LQR
            A = gui_data.A_aug;
            B = gui_data.B_aug;
            C = gui_data.C_aug;
            D = gui_data.D_aug;
            K = gui_data.lqr_gains;
            
            % Sistema en lazo cerrado: G_cl = C*(sI - (A - B*K))^(-1)*B
            A_cl = A - B*K;
            G_controller = ss(A_cl, B, C, D);
            
            % Convertir a función de transferencia para el Bode plot
            try
                G_controller = tf(G_controller);
                % Tomar solo la salida del péndulo (salida 3)
                G_controller = G_controller(3,1);
            catch
                % Si falla la conversión, usar aproximación
            end
        end
        
        % Sistema en lazo abierto
        G_open_loop = G_controller * gui_data.Grrs;
        
        % Función de transferencia en lazo cerrado
        % T(s) = L(s) / (1 + L(s)) donde L(s) es el lazo abierto
        G_closed_loop = feedback(G_open_loop, 1);  % Realimentación unitaria negativa
        
        % Alternativa manual (equivalente):
        % G_closed_loop = G_open_loop / (1 + G_open_loop);
        
        % Generar Bode plot para LAZO CERRADO
        omega = logspace(-1, 3, 1000);  % Frecuencias de 0.01 a 1000 rad/s
        
        % CAMBIO: Usar G_closed_loop en lugar de G_open_loop
        [mag, phase, w] = bode(G_closed_loop, omega);
        mag_db = 20*log10(squeeze(mag));
        phase_deg = squeeze(phase);
        
        % Actualizar gráficas
        set(gui_data.line_bode_mag, 'XData', w, 'YData', mag_db);
        set(gui_data.line_bode_phase, 'XData', w, 'YData', phase_deg);
        
        % Ajustar límites
        xlim(gui_data.ax_bode_mag, [omega(1), omega(end)]);
        xlim(gui_data.ax_bode_phase, [omega(1), omega(end)]);
        
        mag_min = min(mag_db) - 10;
        mag_max = max(mag_db) + 10;
        ylim(gui_data.ax_bode_mag, [mag_min, mag_max]);
        ylim(gui_data.ax_bode_phase, [-180, 180]);
        
        % Añadir líneas de referencia
        hold(gui_data.ax_bode_mag, 'on');
        plot(gui_data.ax_bode_mag, [omega(1), omega(end)], [0, 0], 'k--', 'LineWidth', 0.5);
        plot(gui_data.ax_bode_mag, [omega(1), omega(end)], [max(mag_db)-3, max(mag_db)-3], 'r--', 'LineWidth', 0.5); % Línea -3dB
        hold(gui_data.ax_bode_mag, 'off');
        
        hold(gui_data.ax_bode_phase, 'on');
        plot(gui_data.ax_bode_phase, [omega(1), omega(end)], [-90, -90], 'r--', 'LineWidth', 0.5);
        plot(gui_data.ax_bode_phase, [omega(1), omega(end)], [0, 0], 'k--', 'LineWidth', 0.5);
        hold(gui_data.ax_bode_phase, 'off');
        
        % ANÁLISIS ESPECÍFICO PARA LAZO CERRADO
        try
            % Ancho de banda (-3dB)
            [mag_interp] = interp1(w, mag_db, omega);
            bw_idx = find(mag_interp <= (max(mag_db) - 3), 1, 'first');
            if ~isempty(bw_idx)
                bandwidth = omega(bw_idx);
                fprintf('=== ANÁLISIS LAZO CERRADO ===\n');
                fprintf('Ancho de banda (-3dB): %.2f rad/s\n', bandwidth);
                
                % Marcar en la gráfica
                hold(gui_data.ax_bode_mag, 'on');
                plot(gui_data.ax_bode_mag, bandwidth, max(mag_db)-3, 'gx', 'MarkerSize', 5, 'LineWidth', 1);
                hold(gui_data.ax_bode_mag, 'off');
            end
            
            % Ganancia DC (estado estacionario)
            dc_gain_db = mag_db(1);
            fprintf('Ganancia DC: %.2f dB\n', dc_gain_db);
            
            % Resonancia (pico en magnitud)
            [peak_mag, peak_idx] = max(mag_db);
            peak_freq = w(peak_idx);
            if peak_mag > dc_gain_db + 1  % Solo si hay un pico significativo
                fprintf('Pico de resonancia: %.2f dB en %.2f rad/s\n', peak_mag, peak_freq);
                
                % Marcar en la gráfica
                hold(gui_data.ax_bode_mag, 'on');
                plot(gui_data.ax_bode_mag, peak_freq, peak_mag, 'mx', 'MarkerSize', 5, 'LineWidth', 1);
                hold(gui_data.ax_bode_mag, 'off');
            end
            
        catch ME
            fprintf('Advertencia en análisis: %s\n', ME.message);
        end
        
        controller_name = {'PID', 'LQR'};
        fprintf('Bode plot LAZO CERRADO generado para controlador %s\n', controller_name{gui_data.current_controller + 1});
        
    catch ME
        msgbox(['Error generando Bode plot: ' ME.message], 'Error', 'error');
        fprintf('Error en Bode plot: %s\n', ME.message);
    end
end
    
%% Limpiar gráficas Step y Bode
function clear_analysis(~, ~)
    global gui_data;
    
    % Limpiar datos de step response
    gui_data.step_response_data = [];
    gui_data.step_time_data = [];
    gui_data.step_response_active = false;
    
    % Limpiar gráficas de análisis
    set(gui_data.step_line_response, 'XData', NaN, 'YData', NaN);
    set(gui_data.line_bode_mag, 'XData', NaN, 'YData', NaN);
    set(gui_data.line_bode_phase, 'XData', NaN, 'YData', NaN);
    
    % Restaurar límites
    xlim(gui_data.ax6, [0, 10]);
    ylim(gui_data.ax6, [0, 360]);
    
    % Restaurar botón
    set(gui_data.step_response_btn, 'BackgroundColor', [0.2, 0.6, 0.8], 'String', 'Step Response');
    
    fprintf('Análisis limpiado\n');
end

%% Callbacks principales 
% Seleccionar controlador
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

%% Conección de la comunicación STM-Matlab
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
            set([gui_data.start_btn, gui_data.stop_btn, gui_data.home_btn, gui_data.save_btn, ...
                 gui_data.step_mode_btn, gui_data.step_response_btn, gui_data.freq_response_btn, ...
                 gui_data.clear_analysis_btn], 'Enable', 'on');
            
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
                
                fprintf('Arduino configurado para control directo\n');
                
            catch ME
                fprintf('Advertencia configurando Arduino: %s\n', ME.message);
            end
            
            fprintf('=== CONECTADO EXITOSAMENTE ===\n');
            fprintf('Puerto: %s | Baudrate: 500000\n', port);
            fprintf('Sistema listo para control directo y análisis\n');
            
        else
            msgbox('Error al conectar con Arduino. Verifique el puerto y que esté ejecutando el código correcto.', 'Error', 'error');
        end
    else
        % Desconectar
        disconnect_arduino();
    end
end

%% Actualización para la busqueda de puertos disponibles
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

%% Calcular el valor del LQR
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

%% Iniciar con el control
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
    set(gui_data.system_status, 'String', 'Estado: Control Activo', 'ForegroundColor', [0, 0, 0.8]);
    
    % Iniciar timer
    start(gui_data.timer);
    
    controller_name = {'PID', 'LQR'};
    fprintf('=== CONTROL DIRECTO INICIADO ===\n');
    fprintf('Controlador: %s\n', controller_name{gui_data.current_controller + 1});
    fprintf('IMPORTANTE: Posicione manualmente el péndulo cerca de 180° para mejor control\n');
end

%% Detener el control
function stop_control(~, ~)
    global gui_data;
    
    if gui_data.is_connected
        base_action = [0, 0, 0, 0, 0, 0];
        gui_data.ctrl.step(gui_data.ctrl.CMD_STOP_CONTROL, base_action);
    end
    
    % Detener step response si está activo
    if gui_data.step_response_active
        stop_step_response();
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

%% Establecer la posición actual como el origen
function set_home(~, ~)
    global gui_data;
    
    if gui_data.is_connected && ~gui_data.is_running
        base_action = [0, 0, 0, 0, 0, 0];
        gui_data.ctrl.step(gui_data.ctrl.CMD_SET_HOME, base_action);
        fprintf('Posición home establecida\n');
    end
end

%% Configurar siempre para la mayor cantidad de pasos - Stepper
function set_step_mode(~, ~)
    global gui_data;
    
    if gui_data.is_connected && ~gui_data.is_running
        base_action = [0, 0, 0, 0, 0, 0];
        base_action(1) = 4; % Modo 1/16
        gui_data.ctrl.step(gui_data.ctrl.CMD_SET_STEP_MODE, base_action);
        fprintf('Modo de pasos configurado: 1/16\n');
    end
end

%% Guardar datos y una imagen de las gráficas
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
        
        % Añadir datos de análisis
        if ~isempty(gui_data.step_response_data)
            data_struct.step_response_data = gui_data.step_response_data;
            data_struct.step_time_data = gui_data.step_time_data;
        end
        
        if gui_data.current_controller == 0
            data_struct.pid_gains = gui_data.pid_gains;
        else
            data_struct.lqr_gains = gui_data.lqr_gains;
        end
        
        % Guardar archivo
        timestamp = datestr(now, 'yyyymmdd_HHMMSS');
        controller_name = {'PID', 'LQR'};
        filename = sprintf('pendulum_analysis_%s_%s.mat', controller_name{gui_data.current_controller + 1}, timestamp);
        
        save(filename, 'data_struct');
        
        % Guardar gráficas
        figure_copy = figure('Visible', 'off', 'Position', [0, 0, 1400, 900]);
        
        % Copiar panel de monitoreo
        subplot_copy1 = copyobj(gui_data.plot_panel, figure_copy);
        set(subplot_copy1, 'Position', [0.05, 0.475, 0.9, 0.5]);
        
        % Copiar panel de análisis
        subplot_copy2 = copyobj(gui_data.analysis_panel, figure_copy);
        set(subplot_copy2, 'Position', [0.05, 0.05, 0.9, 0.4]);
        
        img_filename = sprintf('pendulum_analysis_%s_%s.png', controller_name{gui_data.current_controller + 1}, timestamp);
        saveas(figure_copy, img_filename);
        close(figure_copy);
        
        msgbox(sprintf('Datos guardados:\n%s\n%s', filename, img_filename), 'Guardado', 'info');
        
    catch ME
        msgbox(['Error guardando datos: ' ME.message], 'Error', 'error');
    end
end

%% Actualizar constantemente los sensores para obtener los datos en Matlab
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
                
                % Capturar datos para Step Response si está activo
                if gui_data.step_response_active
                    step_time = current_time - (gui_data.step_start_time * 0.05); % Convertir muestra a tiempo
                    if step_time >= 0
                        gui_data.step_response_data(end+1) = obs(1);
                        gui_data.step_time_data(end+1) = step_time;
                    end
                end
                
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
                if mod(gui_data.sample_counter, 50) == 0
                    fprintf('Muestra %d: θ=%.1f°, φ=%.1f°, Status=%d\n', ...
                        gui_data.sample_counter, obs(1), obs(2), status);
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

%% Actualizar el estado del sistema
function update_status_info(obs, status, current_time)
    global gui_data;
    
    theta = obs(1);
    error_integral = obs(6);
    
    % Actualizar displays
    set(gui_data.time_display, 'String', sprintf('Tiempo: %.1f s', current_time));
    set(gui_data.angle_display, 'String', sprintf('Ángulo θ: %.1f°', theta));
    set(gui_data.error_display, 'String', sprintf('Error: %.3f', error_integral));
    
    % Verificar si está invertido
    distance_from_180 = abs(theta - 180.0);
    if distance_from_180 > 180.0
        distance_from_180 = 360.0 - distance_from_180;
    end
    is_upright = distance_from_180 <= 15.0;
    
    if is_upright
        set(gui_data.upright_display, 'String', 'Invertido: SÍ', 'ForegroundColor', [0, 0.8, 0]);
    else
        set(gui_data.upright_display, 'String', 'Invertido: NO', 'ForegroundColor', [0.8, 0, 0]);
    end
    
    % Actualizar según estado
    switch status
        case gui_data.STATUS_OK
            set(gui_data.system_status, 'String', 'Estado: Sistema Listo', 'ForegroundColor', [0, 0.6, 0]);
            
        case gui_data.STATUS_STP_MOVING
            set(gui_data.system_status, 'String', 'Estado: Motor en Movimiento', 'ForegroundColor', [0.6, 0.6, 0]);
            
        case gui_data.STATUS_CONTROL_ACTIVE
            controller_name = {'PID', 'LQR'};
            set(gui_data.system_status, 'String', sprintf('Estado: Control %s Activo', controller_name{gui_data.current_controller + 1}), 'ForegroundColor', [0, 0, 0.8]);
            
        case gui_data.STATUS_UPRIGHT_ACHIEVED
            set(gui_data.system_status, 'String', '¡PÉNDULO INVERTIDO CONTROLADO!', 'ForegroundColor', [0, 0.8, 0]);
    end
end

%% Actualizar todas las gráficas con los datos más recientes 
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
        
        % 1. Ángulo del péndulo con setpoint
        if length(gui_data.theta_data) >= idx_end
            theta_data = gui_data.theta_data(idx_start:idx_end);
            if ~isempty(theta_data) && length(theta_data) == length(x_data)
                setpoint_data = 180 * ones(size(theta_data));
                set(gui_data.line1a, 'XData', x_data, 'YData', setpoint_data);
                set(gui_data.line1b, 'XData', x_data, 'YData', theta_data);
                xlim(gui_data.ax1, [min(x_data)-1, max(x_data)+1]);
                
                % Ajustar límites Y dinámicamente
                theta_min = min(theta_data);
                theta_max = max(theta_data);
                if theta_max > theta_min
                    margin = (theta_max - theta_min) * 0.1;
                    ylim(gui_data.ax1, [max(0, theta_min - margin), min(360, theta_max + margin)]);
                end
            end
        end
        
        % 2. Ángulo del rotor
        if length(gui_data.phi_data) >= idx_end
            phi_data = gui_data.phi_data(idx_start:idx_end);
            if ~isempty(phi_data) && length(phi_data) == length(x_data)
                set(gui_data.line2, 'XData', x_data, 'YData', phi_data);
                xlim(gui_data.ax2, [min(x_data)-1, max(x_data)+1]);
                
                % Ajustar límites Y dinámicamente
                phi_min = min(phi_data);
                phi_max = max(phi_data);
                if phi_max > phi_min
                    phi_range = phi_max - phi_min;
                    margin = max(phi_range * 0.1, 10);
                    ylim(gui_data.ax2, [phi_min - margin, phi_max + margin]);
                end
            end
        end
        
        % 3. Velocidades angulares
        if length(gui_data.dtheta_data) >= idx_end && length(gui_data.dphi_data) >= idx_end
            dtheta_data = gui_data.dtheta_data(idx_start:idx_end);
            dphi_data = gui_data.dphi_data(idx_start:idx_end);
            if ~isempty(dtheta_data) && ~isempty(dphi_data) && ...
               length(dtheta_data) == length(x_data) && length(dphi_data) == length(x_data)
                set(gui_data.line3a, 'XData', x_data, 'YData', dtheta_data);
                set(gui_data.line3b, 'XData', x_data, 'YData', dphi_data);
                xlim(gui_data.ax3, [min(x_data)-1, max(x_data)+1]);
                
                % Ajustar límites Y para velocidades
                all_vel_data = [dtheta_data(:); dphi_data(:)];
                vel_min = min(all_vel_data);
                vel_max = max(all_vel_data);
                if vel_max > vel_min
                    vel_range = vel_max - vel_min;
                    margin = max(vel_range * 0.1, 50);
                    ylim(gui_data.ax3, [vel_min - margin, vel_max + margin]);
                end
            end
        end
        
        % 4. Controlador activo
        if length(gui_data.controller_data) >= idx_end
            controller_data = gui_data.controller_data(idx_start:idx_end);
            if ~isempty(controller_data) && length(controller_data) == length(x_data)
                set(gui_data.line4, 'XData', x_data, 'YData', controller_data);
                xlim(gui_data.ax4, [min(x_data)-1, max(x_data)+1]);
            end
        end
        
        % 5. Step Response (si hay datos)
        if ~isempty(gui_data.step_response_data) && ~isempty(gui_data.step_time_data)
            set(gui_data.step_line_response, 'XData', gui_data.step_time_data, 'YData', gui_data.step_response_data);
            
            if length(gui_data.step_time_data) > 1
                xlim(gui_data.ax6, [0, max(gui_data.step_time_data) + 1]);
                
                step_min = min(gui_data.step_response_data);
                step_max = max(gui_data.step_response_data);
                if step_max > step_min
                    margin = (step_max - step_min) * 0.1;
                    ylim(gui_data.ax6, [step_min - margin, step_max + margin]);
                end
            end
        end
        
        % Forzar actualización
        drawnow;
        
    catch ME
        fprintf('Error actualizando gráficas: %s\n', ME.message);
    end
end

%% Limpiar datos 
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

%% Verificar la conección con el controlador
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
    set([gui_data.start_btn, gui_data.stop_btn, gui_data.home_btn, gui_data.save_btn, ...
         gui_data.step_mode_btn, gui_data.step_response_btn, gui_data.freq_response_btn, ...
         gui_data.clear_analysis_btn], 'Enable', 'off');
    
    fprintf('=== DESCONECTADO DEL DISPOSITIVO ===\n');
end

%% Cerrar la ventana 
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