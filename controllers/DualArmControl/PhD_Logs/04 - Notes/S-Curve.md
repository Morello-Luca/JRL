# S-Curve (Limited jerk)

Se analizziamo un movimento dal punto di vista matematico, abbiamo una gerarchia di derivate rispetto al tempo:

- Posizione ($s$)
- Velocità ($v = \frac{ds}{dt}$)
- Accelerazione ($a = \frac{dv}{dt}$)
- Jerk ($j = \frac{da}{dt}$) — 

ovvero la rapidità con cui varia l'accelerazione.

Nel classico profilo trapezoidale (molto comune ma più brusco), l'accelerazione passa istantaneamente da zero al suo valore massimo. Questo cambio istantaneo genera un jerk infinito (un "colpo" o strattone).

Nel profilo S-Curve, invece, il jerk viene limitato a un valore finito. L'accelerazione non cambia istantaneamente, ma sale e scende linearmente seguendo delle rampe. Di conseguenza:

- La curva della velocità assume una morbida forma a "S" all'inizio e alla fine del movimento.

- Si eliminano i transitori bruschi, riducendo drasticamente le vibrazioni meccaniche.

## Quando si usa il profilo S-Curve?

Il profilo S-Curve viene preferito a quello trapezoidale in tutte le situazioni in cui la fluidità, la precisione e la salvaguardia della meccanica sono prioritari rispetto alla pura semplicità di calcolo.

### 1. Trasporto di carichi sospesi, liquidi o instabili

Se devi spostare un oggetto che può oscillare o cadere facilmente, la S-Curve è d'obbligo.

- Esempi: Carrelli elevatori automatici (AGV/AMR), gru industriali (carroponte), macchine riempitrici di liquidi (flaconi, bottiglie) dove si vuole evitare lo sversamento del prodotto, o trasporto di wafer di silicio nell'industria dei semiconduttori.

### 2. Sistemi ad alta precisione e posizionamento rapido

Nelle macchine utensili o nei sistemi di assemblaggio, le vibrazioni residue a fine movimento allungano il tempo di assestamento (settling time). Se la struttura vibra, il sensore o la testa di stampa devono "aspettare" che si fermi prima di agire.

- Esempi: Macchine di pick-and-place per componenti elettronici, stampanti 3D professionali, macchine per il taglio laser o waterjet. Riducendo il jerk, la macchina si ferma istantaneamente senza oscillare, aumentando la produttività reale (frequenza dei cicli).

### 3. Preservazione della meccanica (Riduzione dell'usura)

I picchi di jerk (accelerazione istantanea) si traducono in forze d'urto sulle componenti meccaniche.

- Esempi: Riduttori epicicloidali, cinghie di trasmissione, viti a ricircolo di sfere e cuscinetti. L'uso della S-Curve riduce lo stress meccanico, i giochi d'inversione (backlash) e prolunga significativamente la vita utile della macchina, riducendo i costi di manutenzione.

### 4. Robotica antropomorfa e collaborativa (Cobot)

I robot che lavorano a stretto contatto con gli esseri umani o che devono compiere traiettorie nello spazio cartesiano utilizzano le S-Curve per due motivi:

- Sicurezza e fluidità: I movimenti fluidi sono più prevedibili e sicuri per l'operatore umano.

- Traiettoria fluida: Evita che i giunti del robot subiscano contraccolpi che danneggerebbero i motori o farebbero scattare i sistemi di sicurezza per sovracorrente.

### 5. Ascensori e trasporto persone

Nel settore civile, la S-Curve è fondamentale per il comfort biologico. Il corpo umano è estremamente sensibile al jerk (la sensazione di "vuoto allo stomaco" o il sobbalzo quando l'ascensore parte o si ferma). Limitando il jerk, la transizione tra fermo e movimento diventa quasi impercettibile.

## Perché usare la S-Curve per una reference di forza?

In un controllore di forza (ad esempio per un attuatore idraulico, un motore lineare o un cobot in controllo di impedenza), la derivata della forza nel tempo ($\frac{dF}{dt}$) è chiamata regime di rampa di forza (o force rate).

- Se applichi un gradino di forza, rischi di generare instabilità, rimbalzi o danneggiare il sistema/l'operatore.

- Limitare il jerk sulla forza significa limitare la derivata seconda della forza ($\frac{d^2F}{dt^2}$). Questo garantisce che la forza non solo salga linearmente, ma che inizi e finisca la rampa in modo incredibilmente morbido.

Dal punto di vista della stabilità del loop di controllo, è una scelta eccellente.

## 2. Analisi critica dei tuoi tre vincoli

Vincolo A: "La reference cambia continuamente in tempo reale"Questo è il punto più critico. Gli algoritmi di S-Curve tradizionali sono progettati per calcolare una traiettoria offline (da un punto $A$ a un punto $B$ noti a priori, partendo da fermo).

- Il problema: Se l'operatore muove continuamente il target, ricalcolare una S-Curve classica da zero a ogni millisecondo è computazionalmente pesante e può portare a comportamenti instabili (il profilo non "insegue" bene il target ma continua a ricalcolare rampe che non finiscono mai).

- La soluzione: Non devi usare un generatore di profili geometrico classico. Devi usare un filtro di traiettoria a jerk limitato in tempo reale (come un filtro lineare di terzo ordine o algoritmi di online trajectory generation tipo l'algoritmo di Reflexxes). In questo modo, lo stato corrente (forza, derivata prima e derivata seconda attuali) viene usato come condizione iniziale per il passo successivo, garantendo continuità assoluta.

## Vincolo B: "Il target può essere aggiornato mentre il profilo è ancora in corso"

Questo in gergo si chiama "trajectory modification on-the-fly".

- Il problema: Se sei a metà di una rampa di accelerazione della forza e il target improvvisamente si dimezza o si inverte, il sistema non può fermarsi istantaneamente perché hai imposto un limite al "jerk" (ovvero a quanto velocemente puoi invertire l'accelerazione). Il sistema mostrerà un inevitabile overshoot (supererà momentaneamente il nuovo target) per poter rallentare nel rispetto dei limiti di derivata prima e seconda che hai impostato.

- La soluzione: L'algoritmo deve essere in grado di calcolare la traiettoria di frenata/inversione più rapida possibile compatibilmente con i limiti fisici. Matematicamente è un problema di controllo ottimo in tempo reale. Se usi una S-Curve semplificata, rischi che il sistema diventi "pigro" o instabile quando si cambia direzione repentinamente.

## Vincolo C: "Vuoi garantire esplicitamente che la rapidità con cui cambia la forza sia limitata, indipendentemente dal target"

Questo è il vero punto di forza del tuo approccio ed è il motivo principale per cui dovresti farlo.

- Definendo dei limiti rigidi su $F_{max}$, $\left(\frac{dF}{dt}\right)_{max}$ e $\left(\frac{d^2F}{dt^2}\right)_{max}$, crei uno scudo matematico per il tuo sistema.

- Qualsiasi "follia" faccia l'operatore sul joystick o sul target (es. un salto istantaneo da $0$ a $1000\text{ N}$), il filtro S-Curve digerirà questo input brusco e lo trasformerà in una rampa dolcissima e sicura. Proteggi i sensori di forza, gli amplificatori e l'hardware da shock meccanici o saturazioni.


## Parte 1: S-Curve sulla Forza vs Il tuo sistema attuale (tanh + low-pass)

Attualmente il tuo codice gestisce la dinamica tramite un filtro passa-basso sulla forza misurata (lambdaFiltered_lowpass_cut) e una funzione tanh sull'errore.

Se sostituisci (o sposti a monte) questo approccio con una S-Curve in tempo reale sulla reference di forza, ottieni vantaggi e svantaggi netti:

- Il vantaggio critico (Perché farlo): La tanh attuale limita l'ampiezza massima della correzione (Kor), ma non limita la derivata. Se l'errore di forza salta istantaneamente, l'output balza al massimo valore consentito dalla tanh in un singolo millisecondo. Questo è un gradino. La S-Curve, invece, garantisce che anche a fronte di un errore catastrofico o di un cambio di target immediato, la forza richiesta ai robot salirà con un'accelerazione (jerk) fluida, eliminando gli shock meccanici sui giunti e sulle celle di carico.

- Lo svantaggio critico (Il compromesso): La S-Curve introduce intrinsecamente un ritardo di fase (delay) maggiore rispetto a un filtro classico o a una tanh. Se l'operatore richiede una forza immediata per afferrare un oggetto che sta scivolando, la S-Curve "frenerà" questa richiesta per rispettare i limiti di jerk, rischiando di far cadere l'oggetto se i limiti sono troppo stringenti.

## Parte 2: Usare l'accelerazione della S-Curve dell'oggetto per la forza "On Demand"

Questo è il punto più interessante del tuo ragionamento. Se anche l'oggetto si muove seguendo un profilo a S-Curve, puoi assolutamente usare la sua accelerazione nominale (desiderata) per calcolare la forza interna. Anzi, in robotica industriale è la best practice per il controllo in feedforward.

Tuttavia, per vincolare l'accelerazione alla forza, devi ribaltare il punto di vista matematico. Non devi solo subire l'accelerazione, devi progettarla in base a quanta forza hai a disposizione.

### Il vincolo fisico bidirezionale

Nel momento in cui usi la S-Curve dell'oggetto per calcolare la forza di presa, scopri che i limiti cinematici dell'oggetto (la sua accelerazione massima) e i limiti attuativi dei robot (la forza massima che possono imprimere per stringere) sono legati dal coefficiente d'attrito ($\mu$):

- Dalla traiettoria alla forza (Il tuo piano): Se l'oggetto accelera bruscamente (ma entro i limiti della sua S-Curve), la traiettoria notifica il controllore di forza che calcola: “Ok, l'oggetto sta accelerando a $5 \text{ m/s}^2$, quindi devo applicare subito $X$ Newton di squeeze per non farlo scivolare”.

- Dalla forza alla traiettoria (Il limite nascosto): Cosa succede se l'oggetto richiede un'accelerazione tale per cui la forza interna necessaria supera gli $80\text{ N}$ (il limite rigido che hai imposto nel tuo codice)? L'oggetto scivolerà, perché la forza satura ma l'accelerazione continua a salire.

### La soluzione ideale: Il Vincolo Dinamico Coerente

Per far funzionare questo sistema in modo impeccabile e sicuro, la S-Curve dell'oggetto e quella della forza devono condividere gli stessi identici limiti temporali (i tempi di rampa).Poiché la S-Curve dell'oggetto ha un profilo ben preciso dove il profilo di accelerazione è trapezoidale (o parabolico), la forza "on demand" (che è proporzionale a quell'accelerazione) assumerà esattamente la stessa identica forma dell'accelerazione.

- Se l'accelerazione dell'oggetto sale linearmente (jerk costante), la tua forza interna dovrà salire linearmente.

- Il jerk della forza interna diventa quindi direttamente proporzionale al jerk cinematico dell'oggetto.

### Conclusione Critica:

Generare la forza interna basandosi sull'accelerazione nominale di una S-Curve cinematica è il modo migliore in assoluto per avere stabilità di grasp, perché il controllore di forza sa in anticipo (a livello di millisecondo) quanta inerzia dovrà contrastare, agendo in modalità feedforward prima ancora che l'oggetto scivoli.Però ricorda: i limiti di accelerazione e jerk della traiettoria dell'oggetto devono essere calcolati a tavolino per non richiedere mai una forza superiore a quella che i tuoi robot possono erogare.