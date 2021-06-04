classdef MicrscopeBloodImage < matlab.mixin.SetGet
    % Klasa MicroscopeBoloodImage jest to klasa realizujaca zadanie
    % projektowe z przedmiotu MZMO. Umożliwia ona analizę obrazu
    % krwi spod mikroskopu - detekcję krwinek na obrazie. 
    % Klasa ta udostępnia metody loadImage(), preProcesingImage(),
    % cellDetector() oraz editDetection().
    % Klasa ta jest przeznaczona do funkcjonowania zaimplementowania jej w
    % aplikacji z gui, w związku z czym w wyniu działania metod nie
    % otrzymujemy wykresów (fragmenty kodu zostały zakomentowane). W wyniku
    % takiego zabiegu wszytskiem właściwością ograniczono dostęp do
    % swobodnego przypisywania danych - tylko w wyniku działania porgramu
    % następują przypisania. Użytkownikowi pozostawiono jednak dowolność w
    % wyświetlaniu danych. Właściwość rawImage jest właściwością publiczną
    
    % właściwośc publiczna
    properties
        rawImage            % macierz surowego zdjęcia
    end
    
    % właściwości z ograniczonym dostępem można pobrać ich wartości ale nie
    % można ich nadawać poza odpowiednimi funkcjami
    properties (GetAccess = public, SetAccess = private)
        postProcesingImage  % obraz po post procesingu
        countWBC            % zliczenia WBC
        countRBC            % zliczenia RBC
        grayScaleImage      % gray scale Image
        contrastImage       % contrast Image
        binaryImage         % binary Image
        maskImage           % mask Image
        maskWBC             % maska binarna WBC
        maskRBC             % maska bianra RBC
        rimRBC              % krawedzie maski dla RBC
        rimWBC              % krawedzie maski dla WBC
        cellRBC             % czerwone tło
        cellWBC             % białe tło
        pointsIDX           % macierz przechowująca punkty markerów komórek
    end
    
    methods
        % funckje set i get dla właściwości cellWBC.
        % funkcja set zawiera kontrole błędów
        
        function image = get.cellWBC(obj)
            image = obj.cellWBC;
        end
        
        function set.cellWBC(obj, val)
                obj.cellWBC = val;
        end
        
        % funckje set i get dla właściwości cellRBC.
        % funkcja set zawiera kontrole błędów
        
        function image = get.cellRBC(obj)
            image = obj.cellRBC;
        end
        
        function set.cellRBC(obj, val)
                obj.cellRBC = val;
        end
        
        % funckje set i get dla właściwości rimWBC.
        % funkcja set zawiera kontrole błędów
        
        function image = get.rimWBC(obj)
            image = obj.rimWBC;
        end
        
        function set.rimWBC(obj, val)
                obj.rimWBC = val;
        end
        
        % funckje set i get dla właściwości rimRBC.
        % funkcja set zawiera kontrole błędów
        
        function image = get.rimRBC(obj)
            image = obj.rimRBC;
        end
        
        function set.rimRBC(obj, val)
                obj.rimRBC = val;
        end
        
        % funckje set i get dla właściwości maskWBC.
        % funkcja set zawiera kontrole błędów
        
        function image = get.maskWBC(obj)
            image = obj.maskWBC;
        end
        
        function set.maskWBC(obj, val)
                obj.maskWBC = val;
        end
        
        % funckje set i get dla właściwości maskRBC.
        % funkcja set zawiera kontrole błędów
        
        function image = get.maskRBC(obj)
            image = obj.maskRBC;
        end
        
        function set.maskRBC(obj, val)
                obj.maskRBC = val;
        end
        
        % funckje set i get dla właściwości maskImage.
        % funkcja set zawiera kontrole błędów
        
        function image = get.maskImage(obj)
            image = obj.maskImage;
        end
        
        function set.maskImage(obj, val)
                obj.maskImage = val;
        end
        
        % funckje set i get dla właściwości binaryImage.
        % funkcja set zawiera kontrole błędów
        
        function image = get.binaryImage(obj)
            image = obj.binaryImage;
        end
        
        function set.binaryImage(obj, val)
                obj.binaryImage = val;
        end
        
        % funckje set i get dla właściwości contrastImage.
        % funkcja set zawiera kontrole błędów
        
        function image = get.contrastImage(obj)
            image = obj.contrastImage;
        end
        
        function set.contrastImage(obj, val)
                obj.contrastImage = val;
        end
        
        % funckje set i get dla właściwości grayScaleImage.
        % funkcja set zawiera kontrole błędów
        
        function image = get.grayScaleImage(obj)
            image = obj.grayScaleImage;
        end
        
        function set.grayScaleImage(obj, val)
                obj.grayScaleImage = val;
        end
        
        % funckje set i get dla właściwości pointsIDX.
        % funkcja set zawiera kontrole błędów
        
        function points = get.pointsIDX(obj)
            points = obj.pointsIDX;
        end
        
        function set.pointsIDX(obj, val)
                obj.pointsIDX = val;
        end
        
        % funckje set i get dla właściwości countRBC.
        % funkcja set zawiera kontrole błędów
        
        function count = get.countWBC(obj)
            count = obj.countWBC;
        end
        
        function set.countWBC(obj, val)
                obj.countWBC = val;
        end
        
        % funckje set i get dla właściwości countRBC.
        % funkcja set zawiera kontrole błędów
        
        function count = get.countRBC(obj)
            count = obj.countRBC;
        end
        
        function set.countRBC(obj, val)
                obj.countRBC = val;
        end
        
        % funckje set i get dla właściwości rawImage.
        % funkcja set zawiera kontrole błędów
        
        function image = get.rawImage(obj)
            image = obj.rawImage;
        end
        
        function set.rawImage(obj, val)
            if isnumeric(val)
                obj.rawImage = val;
            else
                msgbox("Image must be a numeric matrix",'Error','error')
            end
        end
        
        % funckje set i get dla właściwości postProcesingImage.
        % funkcja set zawiera kontrole błędów
        
        function image = get.postProcesingImage(obj)
            image = obj.postProcesingImage;
        end
        
        function set.postProcesingImage(obj, val)
            if islogical(val)
                obj.postProcesingImage = val;
            else
                msgbox("Image must be a logical matrix",'Error','error')
            end
        end
        
        function loadImage(obj)
        % loadImage() - metoda umożliwia wczytanie obrazu
            
            %wybranie ścieżki i pliku
            [file, path] = uigetfile('*.jpg'); 
            % kontrola błędów
            if isnumeric(path) || isnumeric(file)
                msgbox("Path or file didn't chose!",'Error','error');
                return
            end
            obj.rawImage = imread([path,'/', file]); %wczytanie pliku
        end
        
        function preProcesingImage(obj)
            % preProcesingImage() - metoda umożliwia wykonanie
            % preprocesingu obrazu tj. zamiane obrazu RGB na obraz w skali
            % szarości, stworzenie maski bitowej oraz usunięcie małych
            % obszarów będącymi artefaktami.
            % W wyniku działania tej metody do właściwości klasy
            % postProcesingImage zostaje zapisany obraz binarny, oraz na
            % ekranie otrzymujemy okno graficzne z obrazami z
            % poszczególnych etapów preprocesingu.
            
            % zamiana na skale szarosci
            grayImage = rgb2gray(obj.rawImage); 
            % poprawa kontrastu
            contrastI = adapthisteq(grayImage); 
            % binaryzacja obrazu
            binaryImage1 = imbinarize(contrastI, graythresh(contrastI));
            % usuniecie "wklęśnięć" w krwinkach czerwonych
            binaryImage2 = bwareaopen(binaryImage1, 1000);
            % negatyw
            negativeImage = ~binaryImage2;
            % Wypelnienie dziur
            negativeImage = imfill(negativeImage,'holes');
            % usunięcie drobnych obiektow
            negativeImage = bwareaopen(negativeImage, 400);
            
            % plaskie struktury morfologiczne
            se = strel('disk',5);
            % usuniecie fagmentow nie należących do struktur
            % morfologicznych
            obj.postProcesingImage = imerode(negativeImage, se);
            
            % przypisanie obrazów do odpowiednich properties
            obj.grayScaleImage = grayImage;
            obj.contrastImage = contrastI;
            obj.binaryImage = binaryImage2;
            
%             % obraz wykresów
%             figure
%             subplot(2,2,1)
%             imshow(grayImage)
%             title("Gray scale Image")
%             subplot(2,2,2)
%             imshow(contrastI)
%             title("Contrast Image")
%             subplot(2,2,3)
%             imshow(binaryImage2)
%             title("Binarize Image")
%             subplot(2,2,4)
%             imshow(obj.postProcesingImage)
%             title("Negative Eroded Image","without small objects")
        end
        
        function cellDetector(obj)
            % cellDetector() - główna metoda klasy, realizująza zadnie
            % detekcji krwinek.
            % Dokonuje ona automatycznej analizy obrazu. W ramach tej
            % analizy dokonywana jest detekcja krwinek i ich kwalifikacja
            % Czy są to krwinki białe czy czerwone. Dodatkowo metoda ta
            % tworzy macierz informacji o położeniu komórek, które
            % przechowywane są następnie we właściwości pointsIDX.
            % W wyniku działania funkcji użytkownik otrzymuje wczytany
            % przez siebie obraz z naniesionymi na niego punktami
            % czerwone gwiazdki to krwinki czerwone a białe to białe.
            % Przed skorzystaniem z tej metody należy najpierw skorzystać z
            % metody preProcesing().
            
            % wybranie maski bialych krwinek
            if isempty(obj.postProcesingImage)
                msgbox("Firstly use preProcesing method.",'Error','error')
            end
            % wyznaczenie najwiekszej komorki
            largestWBC = bwareafilt(obj.postProcesingImage, 1, 'largest');
            % okreslenie jej pola w pikselach
            fieldWBC = nnz(largestWBC);
            
            % maska białych krwinek
            WBC = bwareafilt(obj.postProcesingImage, [0.55*fieldWBC,fieldWBC]);
            % maska czerwonych krwinek
            RBC = bwareafilt(obj.postProcesingImage, [0.1*fieldWBC,0.5*fieldWBC]);
            
            % wypelnienie masek
            WBCfill = imfill(WBC, 'holes');
            RBCfill = imfill(RBC, 'holes');
            
            % wyznaczenie krawedzi wyekstraktowanych komorek
            WBCrim = bwperim(WBCfill);
            RBCrim = bwperim(RBCfill);
            
            % nadanie ramkom odpowiednich kolrow
            % dla krwinek czerwonych
            redRBC = zeros(size(obj.rawImage,1),size(obj.rawImage,2),3);
            redRBC(:,:,1) = 1;
            % dla krwinek białych
            whiteWBC = zeros(size(obj.rawImage,1),size(obj.rawImage,2),3);
            whiteWBC(:,:,:) = 1;
            se = strel('disk', 7);
            WBCfill = imerode(WBCfill,se);
            
            % zapisanie składowych obrazu ostateczengo do właściwości klasy
            obj.maskRBC = RBCfill;
            obj.maskWBC = WBCfill;
            obj.rimRBC = RBCrim;
            obj.rimWBC = WBCrim;
            obj.cellWBC = whiteWBC;
            obj.cellRBC = redRBC;
            
            % markery na krwinkach czerwonych
            % circle HoughTransform - alorymt ktory wyznacza komorki ktore
            % na siebie nachodza
            [center, ~, ~] = imfindcircles(RBCrim,[10 30],...
                'ObjectPolarity','bright','Sensitivity', 0.9, ...
                'EdgeThreshold', 0.1);
            
            % wyznaczenie srodka krwinek białych
            WBC = regionprops('table',WBCfill,'Centroid');
            
            % wpisanie informacji o polozeniu krwinek do wlasciwosci
            obj.pointsIDX = [table2array(WBC); center];
            % dodanie 3 kolumny z informacja w kodzie asci o kolorze
            % krwinek kod dla pierwszej litery koloru czyli r i w
            [row1, ~] = size(table2array(WBC));
            obj.pointsIDX(1:row1,3) = 119;
            obj.pointsIDX(row1+1:end,3) = 114;
            
%             % wyplotowanie obrazu z zaznaczonymi obszarami detekcji komórek
%             obj.figureDetection = figure;
%             imshow(obj.rawImage)
%             hold on
%             h = imshow(whiteWBC);
%             set(h,'AlphaData',WBCrim)
%             h1 = imshow(redRBC);
%             set(h1,'AlphaData',RBCrim)
%             title("Cell group by type.")
%             % wyplotowanie makrekrów
%             plot(center(:,1),center(:,2),'r*')
%             % wyplotowanie bialych krwinek
%             plot(WBC.Centroid(:,1),WBC.Centroid(:,2),'w*');
%             hold off
        end
        
        function editDetection(obj)
            % editDetection() - metoda klasy, umożliwiająca ręczną korekte
            % detekcji krwinek.
            % Metoda ta pozwala na wskazanie przy pomocy kursora myszki, 
            % fragmentu obrazu z naniesionymi punktami informującymi nas o
            % automatycznie wykrytych krwinkach, i dokonaniu w tym miejscu
            % zmiany. Metoda ta pozwala na dodanie znacznika w białych
            % krwinek po wcisnieciu klawisza w, dodaniu wskaznika
            % czerwonych krwinek po wciśnieciu klawisza r, oraz usuniecie
            % dowolnego wskaznika po wcisnięciu klawisza d.
            % Dodatkowo metoda ta
            % tworzy macierz informacji o położeniu komórek, które
            % przechowywane są następnie we właściwości pointsIDX.
            % W wyniku działania funkcji użytkownik otrzymuje wczytany
            % przez siebie obraz z naniesionymi na niego punktami
            % czerwone gwiazdki to krwinki czerwone a białe to białe.
           
            % stworzenie zmiennej pozwalajacej wykonanie petli
            % nieskonczonej
            doWhile = 1;
            % zmienna przechowujące infomracje o położeniu markeru punktów
            m = obj.pointsIDX;
            
            % rozmiar obrazu
            [x,~,~] = size(obj.rawImage);
            
            % stworzenie nowego okna graficznego
            f = figure;
            imshow(obj.rawImage)
            
            % nieskonczona petla umożliwiająca dodawanie i usuwanie
            % znacznikow krwinek
            while (doWhile)
                % zresetowanie wszystkich uchwytów
                clf(1,'reset');
                % wyplotowanie obrazka
                imshow(obj.rawImage)
                title("Edition Blood Detection")
                % instrukcja poslugiwania się narzedziem
                t1 = text(x-31,10,'Press "s" to save and exit when ready.', 'Color', 'blue');
                t2 = text(x-31,20,'Press "r" to add RBC marker.', 'Color', 'blue');
                t3 = text(x-31,30,'Press "w" to add WBC marker.', 'Color', 'blue');
                t4 = text(x-31,40,'Press "d" to delete the nearest marker.', 'Color', 'blue');
                % stworzenie tymczasowego uchwytu do obecnego axesa
                figGCA = gca;
                
                hold on
                % pobranie ilości punktów w wektorze m - ilość markerów
                [mCounter, ~] = size(m);
                % petla odpowiadajaca za wyrysowanie zancznika na wykresie
                for i = 1:mCounter
                    if (m(i,1)>0)
                        % wyciągniecie wartości informujących o położeniu z macierzy m
                        x1 = m(i,1);
                        y1 = m(i,2);
                        % trzecia kolumna wektora m przechowuje infomracje
                        % o literze w kodzie asci
                        switch (m(i,3))
                            % wyplotowanie odpowiedniego znacznika
                            case {'r'}
                                plot(figGCA,x1,y1,'r*');
                            case {'w'}
                                plot(figGCA,x1,y1,'w*');
                        end
                    end
                end
                
                % funckja pozwalajaca pobrac infomracje o polozeniu kursora
                % na wykresie, zwraca ona informacje do trzy kolumnowego
                % wektora x1 - wsp X, y1 - wsp Y, b - infomracja jaki
                % przycisk został wciśniety
                [x1, y1, b] = ginput(1);
                % detekcja wcisniętego klawisza
                switch (b)
                    case {'s'}
                        % zakonczenie edycji wykresu
                        % nadpisanie infomracji o punktach krwinkach
                        obj.pointsIDX = m; 
                        % usunięcie uchwytów i zamknięcie okna
                        delete(t1);
                        delete(t2);
                        delete(t3);
                        delete(t4);
                        delete(f);
                        doWhile = 0;
                    case {'d'}
                        % wcisniecie d powoduje usuniecie najblizszego
                        % markera
                        if (~isempty(m))
                            % wyznaczenie odchylenia standardowego i
                            % sprawdzenie dla której komórki było ono
                            % najmniejsze
                            [~, ind] = min(sqrt((x1-m(:,1)).^2+(y1-m(:,2)).^2));
                            % do zidentyfikowanego wiersza wpisywane są
                            % wartości -1
                            m(ind,:) = -1;
                        end
                    case {'r','w'}
                        % wpisanie nowych współrzędnych punktów do macierzy
                        % m i infomracji o tym jaki przycisk został użyty -
                        % defacto informacja o typie krwinki
                        m(mCounter+1,1) = x1;
                        m(mCounter+1,2) = y1;
                        m(mCounter+1,3) = b;
                    otherwise
                        % zabezpieczenie przed wcsicnieciem zlych
                        % przyciskow
                        msgbox('Incorrect button pressed ...','Information')
                end
            end
        end
        
        function counter(obj)
            % counter() - metoda klasy umożliwiająca podliczenie
            % zdedykowanych komórek krwi, z podziałem na ich typ.
            % Podliczenie komórek dokonywanejest na podstawie wektora
            % obj.pointsIDX. W wyniku działania tej funkcji wyznaczone
            % ilości krwinek są przypisywane do odpowiednich właściwości
            % klasy tj. obj.countRBC oraz obj.countWBC.
            
            if ~isempty(obj.pointsIDX)
                % wybranie idneksow z krwinkami białymi
                idx = obj.pointsIDX(:,3) == 119;
                WBC = obj.pointsIDX(idx,:);
                
                % wybranie indeksow z krwinkami czerwonymi
                idx = obj.pointsIDX(:,3) == 114;
                RBC = obj.pointsIDX(idx,:);
                
                % policzenie ile kolumn - krwinek wykryto
                obj.countRBC = height(RBC);
                obj.countWBC = height(WBC);
            else
                obj.countRBC = 0;
                obj.countWBC = 0;
            end
        end
    end
end