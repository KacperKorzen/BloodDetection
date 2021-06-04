classdef RoiMicroscopeBloodImage < MicrscopeBloodImage
    % Klasa RoiMicroscopeBoloodImage jest to klasa diedzicząca po klasie
    % MicroscopeBloodImage. Klasa ta jest rozbudowana o metode ROI() oraz 
    % report().
    
    methods    
        
        function ROI(obj, xmin, xmax, ymin, ymax)
        % ROI() umożliwia przycięcie obrazu rawImage do danego okna 
        % zainteresowania. Przycięcty obraz jest nadpisywany we właściwosci
        % rawImage klasy.
        % Do metody podajemy 4 argumnety numeryczne, które oznaczają
        % zakresy okna ROI. Argumenty funkcji muszą być numeryczne 
            
            arguments
                % zapewnienie poprawnych wartości argumentów
                obj
                xmin {mustBeNumeric}
                xmax {mustBeNumeric}
                ymin {mustBeNumeric}
                ymax {mustBeNumeric}
            end
            
            % kontrola błędów - argumenty muszą być w zakresie
            [rows,cols, ~] = size(obj.rawImage);
            if xmin < 1 || xmax > cols || ymin < 1 || ymax > rows
                msgbox("Range must be part of rawImage size",'Error','error')
            else
                % przypisanie zmniejszonego obrazu
                obj.rawImage = obj.rawImage(ymin:ymax, xmin:xmax,:);
            end
        end
    end
    
    % metody statyczne
    methods(Static)  
        function val = getPathFile()
            % metoda pozwalajaca pobrać nazwę pliku i ścieżkę do zapisu
            % raportu - wykresu z zaznaczonymi punktami i podliczonymi
            % krwinkami
            
            % pobranie ścieżki
            [file, path] = uiputfile('*.jpg');
            % zabezpieczenie
            if isnumeric(path) || isnumeric(file)
                msgbox("Path or file didn't chose!",'Error','error');
                val = [];
            else
                % zwrócenie ścieżki
                val = [path,file];
            end
        end
    end
end