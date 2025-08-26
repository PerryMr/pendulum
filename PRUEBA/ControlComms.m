classdef ControlComms < handle
    %CONTROLCOMMS Clase para comunicación serial con Arduino
    %   Esta clase maneja la comunicación bidireccional con el controlador
    %   del péndulo invertido mediante protocolo JSON sobre puerto serial
    %
    %   VERSIÓN ACTUALIZADA v1.1:
    %   - Agregados comandos para control directo sin swing-up
    %   - CMD_START_DIRECT_CONTROL: Control inmediato
    %   - CMD_SET_SWING_MODE: Habilitar/deshabilitar swing-up
    %   - STATUS_DIRECT_CONTROL: Nuevo estado de sistema
    
    properties (Access = private)
        serialObj           % Objeto puerto serial
        timeout            % Timeout en segundos
        debugLevel         % Nivel de debug
    end
    
    properties (Constant)
        % Códigos de estado
        STATUS_OK = 0
        STATUS_ERROR = 1
        
        % Niveles de debug
        DEBUG_NONE = 0
        DEBUG_ERROR = 1
        DEBUG_WARN = 2
        DEBUG_INFO = 3
        
        % Comandos para Arduino - ACTUALIZADOS
        CMD_SET_HOME = 0
        CMD_MOVE_TO = 1
        CMD_MOVE_BY = 2
        CMD_SET_STEP_MODE = 3
        CMD_SELECT_CONTROLLER = 4
        CMD_SET_PID_GAINS = 5
        CMD_SET_LQR_GAINS = 6
        CMD_START_CONTROL = 7              % Inicia con swing-up (si está habilitado)
        CMD_STOP_CONTROL = 8
        CMD_SET_SWING_PARAMS = 9
        CMD_START_DIRECT_CONTROL = 10      % NUEVO: Control directo sin swing-up
        CMD_SET_SWING_MODE = 11            % NUEVO: Habilitar/deshabilitar swing-up
        
        % Estados del sistema - ACTUALIZADOS
        STATUS_STP_MOVING = 1
        STATUS_SWING_UP = 2
        STATUS_CONTROL_ACTIVE = 3
        STATUS_UPRIGHT_ACHIEVED = 4
        STATUS_SWING_FORWARD = 5
        STATUS_SWING_BACKWARD = 6
        STATUS_DIRECT_CONTROL = 7          % NUEVO: Control directo activo
        
        % Controladores
        CONTROLLER_PID = 0
        CONTROLLER_LQR = 1
        
        % Claves JSON para transmisión
        TX_KEY_ACTION = 'action'
        TX_KEY_COMMAND = 'command'
        
        % Claves JSON para recepción
        RX_KEY_STATUS = 'status'
        RX_KEY_TIMESTAMP = 'timestamp'
        RX_KEY_TERMINATED = 'terminated'
        RX_KEY_OBSERVATION = 'observation'
    end
    
    methods
        function obj = ControlComms(timeout, debugLevel)
            %CONTROLCOMMS Constructor
            %   timeout - Tiempo de espera en segundos (default: 1.0)
            %   debugLevel - Nivel de debug (default: DEBUG_NONE)
            
            if nargin < 1
                timeout = 1.0;
            end
            if nargin < 2
                debugLevel = obj.DEBUG_NONE;
            end
            
            obj.timeout = timeout;
            obj.debugLevel = debugLevel;
            obj.serialObj = [];
        end
        
        function delete(obj)
            %DELETE Destructor - cierra puerto serial
            obj.close();
        end
        
        function portList = getSerialList(obj)
            %GETSERIALLIST Obtiene lista de puertos seriales disponibles
            %   Retorna cell array con información de puertos
            
            try
                portInfo = serialportlist("available");
                portList = cellstr(portInfo);
            catch
                % Para versiones anteriores de MATLAB
                try
                    portInfo = instrhwinfo('serial');
                    if isfield(portInfo, 'AvailableSerialPorts')
                        portList = portInfo.AvailableSerialPorts;
                    else
                        portList = {};
                    end
                catch
                    portList = {};
                end
            end
        end
        
        function status = connect(obj, port, baudRate)
            %CONNECT Conecta al puerto serial especificado
            %   port - Nombre del puerto (ej: 'COM10')
            %   baudRate - Velocidad de comunicación (default: 115200)
            %   Retorna STATUS_OK o STATUS_ERROR
            
            if nargin < 3
                baudRate = 115200;
            end
            
            % Cerrar conexión previa si existe
            obj.close();
            
            try
                % Crear objeto serial
                if exist('serialport', 'file') == 2
                    % MATLAB R2019b o posterior
                    obj.serialObj = serialport(port, baudRate, ...
                        'Timeout', obj.timeout, ...
                        'DataBits', 8, ...
                        'Parity', 'none', ...
                        'StopBits', 1, ...
                        'FlowControl', 'none');
                else
                    % Versiones anteriores de MATLAB
                    obj.serialObj = serial(port, ...
                        'BaudRate', baudRate, ...
                        'Timeout', obj.timeout, ...
                        'DataBits', 8, ...
                        'Parity', 'none', ...
                        'StopBits', 1, ...
                        'FlowControl', 'none', ...
                        'Terminator', 'LF');
                    fopen(obj.serialObj);
                end
                
                status = obj.STATUS_OK;
                
                if obj.debugLevel >= obj.DEBUG_INFO
                    fprintf('Conectado a %s a %d baudios\n', port, baudRate);
                end
                
            catch ME
                if obj.debugLevel >= obj.DEBUG_ERROR
                    fprintf('Error conectando a puerto serial: %s\n', ME.message);
                end
                status = obj.STATUS_ERROR;
            end
        end
        
        function close(obj)
            %CLOSE Cierra la conexión serial
            
            if ~isempty(obj.serialObj)
                try
                    if exist('serialport', 'file') == 2
                        % MATLAB R2019b o posterior
                        delete(obj.serialObj);
                    else
                        % Versiones anteriores
                        if strcmp(obj.serialObj.Status, 'open')
                            fclose(obj.serialObj);
                        end
                        delete(obj.serialObj);
                    end
                catch ME
                    if obj.debugLevel >= obj.DEBUG_WARN
                        fprintf('Advertencia cerrando puerto: %s\n', ME.message);
                    end
                end
                obj.serialObj = [];
            end
        end
        
        function result = step(obj, command, action)
            %STEP Envía comando y acciones, espera observación
            %   command - Comando entero a enviar
            %   action - Vector de acciones (números flotantes)
            %   Retorna struct con campos: status, timestamp, terminated, observation
            %   o empty si hay error
            
            result = [];
            
            if isempty(obj.serialObj)
                if obj.debugLevel >= obj.DEBUG_ERROR
                    fprintf('Error: Puerto serial no conectado\n');
                end
                return;
            end
            
            % Preparar mensaje JSON
            if isscalar(action)
                actionStr = sprintf('%.6f', action);
            else
                actionStr = sprintf('%.6f,', action);
                actionStr = actionStr(1:end-1); % Remover última coma
            end
            
            msg = sprintf('{"command":%d,"action":[%s]}', command, actionStr);
            
            try
                % Limpiar buffer de entrada antes de enviar
                if exist('serialport', 'file') == 2
                    if obj.serialObj.NumBytesAvailable > 0
                        flush(obj.serialObj);
                    end
                else
                    while obj.serialObj.BytesAvailable > 0
                        fread(obj.serialObj, obj.serialObj.BytesAvailable);
                    end
                end
                
                % Enviar mensaje
                if exist('serialport', 'file') == 2
                    % MATLAB R2019b o posterior
                    write(obj.serialObj, msg, 'string');
                else
                    % Versiones anteriores
                    fprintf(obj.serialObj, '%s\n', msg);
                end
                
                % Esperar respuesta con timeout mejorado
                response = '';
                attempts = 0;
                max_attempts = 10;
                
                while isempty(response) && attempts < max_attempts
                    try
                        if exist('serialport', 'file') == 2
                            response = readline(obj.serialObj);
                            response = char(response);
                        else
                            response = fgetl(obj.serialObj);
                        end
                        
                        if isempty(response) || (ischar(response) && strcmp(response, ''))
                            pause(0.01); % Breve pausa antes de reintentar
                            attempts = attempts + 1;
                        end
                    catch
                        attempts = attempts + 1;
                        pause(0.01);
                    end
                end
                
                if isempty(response) || attempts >= max_attempts
                    if obj.debugLevel >= obj.DEBUG_ERROR
                        fprintf('Timeout: No se recibió respuesta del Arduino\n');
                    end
                    return;
                end
                
                % Validar que la respuesta sea JSON válido
                if ~ischar(response) || isempty(strfind(response, '{'))
                    if obj.debugLevel >= obj.DEBUG_ERROR
                        fprintf('Respuesta inválida (no JSON): %s\n', response);
                    end
                    return;
                end
                
                % Parsear JSON
                data = jsondecode(response);
                
                % Crear estructura de resultado
                result = struct();
                
                % Verificar que los campos existen
                if isfield(data, obj.RX_KEY_STATUS)
                    result.status = data.(obj.RX_KEY_STATUS);
                else
                    result.status = obj.STATUS_ERROR;
                end
                
                if isfield(data, obj.RX_KEY_TIMESTAMP)
                    result.timestamp = data.(obj.RX_KEY_TIMESTAMP);
                else
                    result.timestamp = 0;
                end
                
                if isfield(data, obj.RX_KEY_TERMINATED)
                    result.terminated = data.(obj.RX_KEY_TERMINATED);
                else
                    result.terminated = false;
                end
                
                if isfield(data, obj.RX_KEY_OBSERVATION)
                    result.observation = data.(obj.RX_KEY_OBSERVATION);
                else
                    result.observation = [];
                end
                
            catch ME
                if obj.debugLevel >= obj.DEBUG_ERROR
                    fprintf('Error en comunicación: %s\n', ME.message);
                    if exist('response', 'var') && ~isempty(response)
                        fprintf('Respuesta recibida: %s\n', response);
                    end
                end
            end
        end
        
        function setTimeout(obj, timeout)
            %SETTIMEOUT Establece el timeout de comunicación
            %   timeout - Tiempo en segundos
            
            obj.timeout = timeout;
            if ~isempty(obj.serialObj)
                if exist('serialport', 'file') == 2
                    obj.serialObj.Timeout = timeout;
                else
                    obj.serialObj.Timeout = timeout;
                end
            end
        end
        
        % NUEVOS MÉTODOS PARA FUNCIONALIDAD EXTENDIDA
        
        function result = startSwingUpControl(obj)
            %STARTSWINGUPCONTROL Inicia control con swing-up
            %   Utiliza el comando tradicional CMD_START_CONTROL
            %   El comportamiento depende de la configuración del swing-up en Arduino
            
            base_action = [0, 0, 0, 0, 0, 0];
            result = obj.step(obj.CMD_START_CONTROL, base_action);
            
            if obj.debugLevel >= obj.DEBUG_INFO
                fprintf('Comando enviado: START_CONTROL (con swing-up si está habilitado)\n');
            end
        end
        
        function result = startDirectControl(obj)
            %STARTDIRECTCONTROL Inicia control directo sin swing-up
            %   Utiliza el nuevo comando CMD_START_DIRECT_CONTROL
            %   Va directamente al control asumiendo que el péndulo está cerca de la posición invertida
            
            base_action = [0, 0, 0, 0, 0, 0];
            result = obj.step(obj.CMD_START_DIRECT_CONTROL, base_action);
            
            if obj.debugLevel >= obj.DEBUG_INFO
                fprintf('Comando enviado: START_DIRECT_CONTROL (sin swing-up)\n');
            end
        end
        
        function result = setSwingMode(obj, enabled)
            %SETSWINGMODE Habilita o deshabilita el modo swing-up
            %   enabled - true para habilitar swing-up, false para deshabilitarlo
            %   Controla el comportamiento del comando CMD_START_CONTROL
            
            swing_action = [double(enabled), 0, 0, 0, 0, 0];
            result = obj.step(obj.CMD_SET_SWING_MODE, swing_action);
            
            if obj.debugLevel >= obj.DEBUG_INFO
                if enabled
                    fprintf('Swing-up HABILITADO: CMD_START_CONTROL ejecutará swing-up + control\n');
                else
                    fprintf('Swing-up DESHABILITADO: CMD_START_CONTROL ejecutará control directo\n');
                end
            end
        end
        
        function result = setSwingParameters(obj, forward_angle, backward_angle, forward_speed, backward_speed)
            %SETSWINGPARAMETERS Configura parámetros del swing-up
            %   forward_angle - Ángulo de avance en grados
            %   backward_angle - Ángulo de retroceso en grados  
            %   forward_speed - Velocidad de avance en steps/sec
            %   backward_speed - Velocidad de retroceso en steps/sec
            
            swing_params = [forward_angle, backward_angle, forward_speed, backward_speed, 0, 0];
            result = obj.step(obj.CMD_SET_SWING_PARAMS, swing_params);
            
            if obj.debugLevel >= obj.DEBUG_INFO
                fprintf('Parámetros swing-up actualizados:\n');
                fprintf('  Avance: %.1f° a %.0f steps/sec\n', forward_angle, forward_speed);
                fprintf('  Retroceso: %.1f° a %.0f steps/sec\n', backward_angle, backward_speed);
            end
        end
        
        function statusStr = getStatusString(obj, status)
            %GETSTATUSSTRING Convierte código de estado a string descriptivo
            %   status - Código numérico de estado
            %   Retorna string descriptivo del estado
            
            switch status
                case obj.STATUS_OK
                    statusStr = 'Sistema OK';
                case obj.STATUS_STP_MOVING
                    statusStr = 'Motor en movimiento';
                case obj.STATUS_SWING_UP
                    statusStr = 'Swing-up activo';
                case obj.STATUS_CONTROL_ACTIVE
                    statusStr = 'Control activo (post swing-up)';
                case obj.STATUS_UPRIGHT_ACHIEVED
                    statusStr = '¡Posición invertida lograda!';
                case obj.STATUS_SWING_FORWARD
                    statusStr = 'Swing-up: Fase avance';
                case obj.STATUS_SWING_BACKWARD
                    statusStr = 'Swing-up: Fase retroceso';
                case obj.STATUS_DIRECT_CONTROL
                    statusStr = 'Control directo activo';
                otherwise
                    statusStr = sprintf('Estado desconocido (%d)', status);
            end
        end
        
        function printSystemInfo(obj)
            %PRINTSYSTEMINFO Imprime información del sistema de comandos
            
            fprintf('=== SISTEMA DE CONTROL PÉNDULO INVERTIDO ===\n');
            fprintf('ControlComms v1.1 - Con soporte para control directo\n\n');
            
            fprintf('COMANDOS DISPONIBLES:\n');
            fprintf('  %d - SET_HOME: Establecer posición home\n', obj.CMD_SET_HOME);
            fprintf('  %d - MOVE_TO: Mover a posición específica\n', obj.CMD_MOVE_TO);
            fprintf('  %d - MOVE_BY: Mover por cantidad relativa\n', obj.CMD_MOVE_BY);
            fprintf('  %d - SET_STEP_MODE: Configurar modo de pasos\n', obj.CMD_SET_STEP_MODE);
            fprintf('  %d - SELECT_CONTROLLER: Seleccionar PID/LQR\n', obj.CMD_SELECT_CONTROLLER);
            fprintf('  %d - SET_PID_GAINS: Configurar ganancias PID\n', obj.CMD_SET_PID_GAINS);
            fprintf('  %d - SET_LQR_GAINS: Configurar ganancias LQR\n', obj.CMD_SET_LQR_GAINS);
            fprintf('  %d - START_CONTROL: Iniciar control (con/sin swing-up)\n', obj.CMD_START_CONTROL);
            fprintf('  %d - STOP_CONTROL: Detener control\n', obj.CMD_STOP_CONTROL);
            fprintf('  %d - SET_SWING_PARAMS: Configurar parámetros swing-up\n', obj.CMD_SET_SWING_PARAMS);
            fprintf('  %d - START_DIRECT_CONTROL: Control directo sin swing-up\n', obj.CMD_START_DIRECT_CONTROL);
            fprintf('  %d - SET_SWING_MODE: Habilitar/deshabilitar swing-up\n', obj.CMD_SET_SWING_MODE);
            
            fprintf('\nESTADOS DEL SISTEMA:\n');
            fprintf('  %d - OK: Sistema listo\n', obj.STATUS_OK);
            fprintf('  %d - STP_MOVING: Motor en movimiento\n', obj.STATUS_STP_MOVING);
            fprintf('  %d - SWING_UP: Swing-up en progreso\n', obj.STATUS_SWING_UP);
            fprintf('  %d - CONTROL_ACTIVE: Control activo (post swing-up)\n', obj.STATUS_CONTROL_ACTIVE);
            fprintf('  %d - UPRIGHT_ACHIEVED: ¡Posición invertida lograda!\n', obj.STATUS_UPRIGHT_ACHIEVED);
            fprintf('  %d - SWING_FORWARD: Swing-up fase avance\n', obj.STATUS_SWING_FORWARD);
            fprintf('  %d - SWING_BACKWARD: Swing-up fase retroceso\n', obj.STATUS_SWING_BACKWARD);
            fprintf('  %d - DIRECT_CONTROL: Control directo activo\n', obj.STATUS_DIRECT_CONTROL);
            
            fprintf('\nMODOS DE OPERACIÓN:\n');
            fprintf('1. SWING-UP + CONTROL:\n');
            fprintf('   - Usar setSwingMode(true) + startSwingUpControl()\n');
            fprintf('   - Ejecuta swing-up automático seguido de control\n');
            fprintf('2. CONTROL DIRECTO:\n');
            fprintf('   - Usar startDirectControl() directamente\n');
            fprintf('   - Para péndulos ya cerca de posición invertida\n');
            fprintf('3. MODO HÍBRIDO:\n');
            fprintf('   - Usar setSwingMode(false) + startSwingUpControl()\n');
            fprintf('   - CMD_START_CONTROL actuará como control directo\n');
            
            fprintf('\n=== FIN INFORMACIÓN SISTEMA ===\n');
        end
    end
end