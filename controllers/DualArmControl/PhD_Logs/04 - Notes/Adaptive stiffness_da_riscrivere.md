# Stiffness Adattiva Force-Aware per Manipolazione Dual-Arm Collaborativa

## Obiettivo del documento

Oggi, in `stateCollaborative` / `COOPERATIVE_MOTION`, la stiffness dei due `ImpedanceTask` è hardcodata in modo asimmetrico:

```cpp
Eigen::Matrix<double, 6, 1> springGainsRight { 10.0, 10.0, 10.0, 150.0, 150.0, 150.0 };
Eigen::Matrix<double, 6, 1> springGainsLeft  { 10.0, 10.0, 10.0, 50.0,  50.0,  50.0  };
```

Il braccio sinistro è reso più cedevole a priori perché, nella traslazione laterale di questo specifico task, il suo errore di posizione e la forza di contatto risultano sistematicamente in direzioni opposte: se lo lasci rigido, la molla virtuale combatte contro il moto imposto dal braccio destro invece di assecondarlo.

Il documento risponde a tre domande, in ordine:

1. **Cosa succede matematicamente** quando errore e forza sono discordi, e perché questo giustifica l'abbassamento di stiffness.
2. **Quali politiche di adattamento** sono valide (gradiente, tanh, o entrambe combinate) e come si formalizzano.
3. **QP istantaneo vs MPC**: quale architettura è compatibile con il codice che hai oggi, e cosa cambierebbe se volessi passare a un MPC.

Alla fine trovi uno pseudocodice pronto da adattare, i punti di integrazione esatti nel tuo file, e un piano di validazione.

---

## 1. Richiamo del modello fisico attuale

### 1.1 Grasp frame e scomposizione delle forze (già implementato)

In `buildGraspFrame()` costruisci il frame oggetto `X_world_object` a metà tra i due end-effector, con asse x lungo la congiungente `pR - pL`. Da lì costruisci la grasp matrix:

$$
G = \begin{bmatrix} I_3 & 0 & I_3 & 0 \\ [r_L]_\times & I_3 & [r_R]_\times & I_3 \end{bmatrix} \in \mathbb{R}^{6\times12}
$$

che mappa le wrench dei due end-effector nella wrench risultante sull'oggetto (forza + momento netti). La pseudoinversa `Gpinv_` ti dà la mappa inversa minima-norma. Questo è il canale giusto per separare **forza interna** (grasping/squeeze) da **forza esterna netta** (quella che muove l'oggetto) — è la stessa logica che userai per il segnale di adattamento, quindi non stai introducendo un concetto nuovo, stai riusando quello che hai già.

### 1.2 Legge di impedenza per singolo braccio

Ogni `ImpedanceTask` realizza, concettualmente, per l'asse i-esimo:

$$
F_{\text{cmd},i} = K_i \, e_i + D_i \, \dot e_i + F_{\text{target},i}
$$

dove:
- $e_i = x_{\text{des},i} - x_{\text{meas},i}$ è l'errore di posizione/orientamento sull'asse i (nel tuo caso, ordine sva: 0:3 angolare, 3:6 lineare),
- $K_i$ è la stiffness virtuale (quello che oggi è hardcodato in `springGainsLeft/Right`),
- $F_{\text{target}}$ è il feed-forward di wrench (usato per la fase di approach ramp e per `gains.wrenchGains`).

### 1.3 Il fenomeno che vuoi catturare

Definisci per ciascun braccio, sull'asse (o sottospazio) di interesse:

- errore di posizione $e_i \in \mathbb{R}^3$ (o scalare, se proietti su un solo asse),
- forza di contatto misurata $f_i = \text{measuredWrench().force()}$.

Il problema che descrivi è: quando $e_i$ e $f_i$ **puntano in direzioni opposte**, la molla virtuale $K_i e_i$ genera una forza di comando che *si oppone* alla forza esterna misurata invece di assecondarla — il braccio "tira" contro quello che il contatto sta già facendo. Vuoi che in quella condizione $K_i$ si abbassi, rendendo il braccio compliant.

Il modo naturale per formalizzare "direzioni opposte" è la **cosine similarity** tra i due vettori:

$$
s_i = \frac{e_i \cdot f_i}{\lVert e_i \rVert \, \lVert f_i \rVert + \epsilon} \in [-1, 1]
$$

- $s_i \to -1$: errore e forza opposti → vuoi $K_i \to K_{\min}$ (compliant).
- $s_i \to +1$: errore e forza allineati (il braccio spinge nella stessa direzione della forza, sinergico) → puoi tenere $K_i \to K_{\max}$ (rigido, buon tracking).
- $\epsilon$ piccolo (es. 1e-3) evita divisione per zero quando forza o errore sono ~nulli.

**Nota pratica sull'asse di proiezione**: dato che il fenomeno che descrivi è specificamente sulla traslazione laterale (l'asse che congiunge i due end-effector, cioè l'asse x del grasp frame), ti conviene **non** fare la cosine similarity sul vettore 3D completo, ma proiettare $e$ e $f$ sull'asse locale di interesse (x del `X_world_object`, o l'asse mondo se preferisci lavorare in world frame) prima di fare il rapporto. Questo evita che una discrepanza su un asse ortogonale (es. verticale, dove hai il feed-forward di `wrenchGains`) inquini il segnale di adattamento pensato per la lateralità. Se in futuro ti serve il comportamento su più assi contemporaneamente, puoi calcolare $s_i$ per ciascun asse separatamente (stiffness diagonale, non scalare).

---

## 2. Politiche di adattamento: gradiente, tanh, o entrambe

Non sono due alternative mutuamente esclusive — sono due pezzi complementari di uno stesso schema, e ti conviene usarli insieme:

- il **tanh** definisce la *mappa statica* (quanto vale la stiffness target, dato l'allineamento attuale),
- il **gradiente** (in forma di filtro del primo ordine) definisce la *dinamica* con cui la stiffness reale insegue quel target, evitando salti bruschi che destabilizzerebbero il QP.

### 2.1 Mappa statica: shaping via tanh

$$
K_{\text{target},i}(s_i) = K_{\min} + \frac{K_{\max}-K_{\min}}{2}\Big(1 + \tanh(\alpha \, s_i)\Big)
$$

Proprietà utili:
- è **limitata** per costruzione tra $K_{\min}$ e $K_{\max}$ — non serve clamping esterno, importante perché il QP non deve mai ricevere guadagni fuori range fisicamente sensato;
- è **liscia** (derivabile ovunque), quindi non introduce discontinuità nel Jacobiano del task quando $s_i$ cambia segno;
- $\alpha$ controlla la "durezza" della transizione: $\alpha$ piccolo → transizione dolce e ampia zona di compromesso attorno a $s_i=0$; $\alpha$ grande → comportamento quasi bang-bang (rigido/compliant), più simile a uno switch. Parti con $\alpha \in [1, 3]$ e regola sperimentalmente.

Alternative equivalenti a tanh, se vuoi confrontare in simulazione:
- **sigmoide logistica** $\sigma(\alpha s_i) = 1/(1+e^{-\alpha s_i})$ — stessa forma a S, meno simmetrica esplicitamente ma equivalente a meno di riscalatura;
- **funzione lineare saturata (clamp)** — più semplice, ma non derivabile ai bordi, sconsigliata dentro un QP che assume regolarità.

Il tanh è la scelta più robusta delle tre: nessun parametro aggiuntivo oltre $\alpha$, saturazione automatica, derivata continua.

### 2.2 Dinamica: legge a gradiente / filtro del primo ordine

Anche con una mappa statica liscia, se $s_i$ oscilla rapidamente (rumore di forza, piccoli errori di stima), $K_{\text{target}}$ può cambiare da un ciclo di controllo all'altro in modo brusco. Nel tuo QP, un salto di stiffness cambia istantaneamente la forza comandata a parità di errore — è esattamente il tipo di discontinuità che può eccitare gli attuatori o causare un "colpo" percepibile sull'oggetto afferrato.

La soluzione standard (coerente con quello che già fai per `lambdaFiltered_lowpass_cut`) è trattare l'adattamento come una discesa a gradiente verso il target, cioè un filtro del primo ordine:

$$
\dot K_i = -\gamma \, \big(K_i - K_{\text{target},i}(s_i)\big)
$$

discretizzato a passo $\Delta t$ (il tuo `timeStep`):

$$
K_i[k+1] = K_i[k] + \gamma \, \Delta t \, \big(K_{\text{target},i}(s_i[k]) - K_i[k]\big)
$$

Questo è letteralmente un passo di gradiente su un costo quadratico $J = \frac{1}{2}(K_i - K_{\text{target},i})^2$, quindi risponde anche formalmente alla tua domanda "gradiente o tanh": il gradiente qui non sostituisce il tanh, lo insegue nel tempo.

Vantaggi:
- $\gamma$ ti dà un secondo grado di libertà, indipendente da $\alpha$, per la **velocità di risposta** (costante di tempo $\tau = 1/\gamma$). Puoi renderlo lento (es. $\tau \approx 0.3$–0.5s) per evitare che rumore ad alta frequenza sulla forza si traduca in stiffness "nervosa".
- Il filtro garantisce $\dot K_i$ limitato in norma se $s_i$ è limitato — puoi anche aggiungere un rate-limit esplicito $|\dot K_i| \le \dot K_{\max}$ come ulteriore rete di sicurezza indipendente dalla scelta di $\gamma$.
- È la stessa struttura concettuale del low-pass che già usi su `lambdaFiltered_lowpass_cut`: stai riusando un pattern che il resto del codice già conosce, invece di introdurre un meccanismo nuovo.

### 2.3 Riassunto: la legge completa proposta

Per ciascun braccio $i \in \{L, R\}$, ad ogni ciclo di `COOPERATIVE_MOTION`:

1. calcola $e_i$ (errore posizione, proiettato sull'asse di interesse) e $f_i$ (forza misurata, stessa proiezione);
2. calcola $s_i = \dfrac{e_i \cdot f_i}{\lVert e_i\rVert \lVert f_i\rVert + \epsilon}$;
3. (opzionale ma consigliato) filtra $s_i$ con un low-pass leggero, per non propagare rumore di forza direttamente nel target;
4. calcola $K_{\text{target},i} = K_{\min} + \frac{K_{\max}-K_{\min}}{2}(1+\tanh(\alpha s_i))$;
5. integra $K_i[k+1] = K_i[k] + \gamma\Delta t (K_{\text{target},i} - K_i[k])$, con eventuale clamp su $\dot K_i$;
6. passa $K_i[k+1]$ a `setImpedanceGains(...)` al posto del valore hardcodato, solo sulle componenti (traslazionali, indici 3:6) coinvolte nel fenomeno — lascia invariati gli assi non toccati dal problema (es. verticale, dove hai già `wrenchGains` a gestire il contatto).

---

## 3. QP istantaneo vs MPC: quale architettura ha senso qui

### 3.1 Cosa hai oggi

Il tuo solver (`mc_rtc`, `solver().addTask(...)`) risolve un QP **istantaneo**, senza orizzonte di predizione: ad ogni ciclo minimizza l'errore dei task correnti dati i guadagni correnti. Non c'è un modello dinamico esplicito del contatto o dell'oggetto afferrato integrato nel solver — la "previsione" che hai oggi è solo il profilo quintico di `computeDesiredObjectPose()`, che è una traiettoria di riferimento pre-pianificata, non un controllo predittivo.

### 3.2 Perché la legge gradiente+tanh è compatibile senza modifiche architetturali

La legge proposta in Sezione 2 è **memoryless rispetto al QP**: ad ogni ciclo calcoli $K_i[k+1]$ fuori dal solver (in un metodo tipo `computeAdaptiveStiffness()`, chiamato prima di `setImpedanceGains`) e lo passi come parametro. Il QP continua a vedere "solo" una stiffness — non sa né gli interessa che sia adattiva. Non tocchi constraint set, non aggiungi variabili di decisione, non cambi il backend. È un layer che sta *sopra* al QP esistente, non dentro.

Questo è il motivo per cui te la consiglio come primo passo: **costo di ingegnerizzazione basso, rischio basso, e osservabile/loggabile subito** (puoi loggare $s_i$, $K_{\text{target}}$, $K_i$ accanto a quello che già logghi con `registerCollaborativeLogs()`).

### 3.3 Quando avrebbe senso un MPC

Un MPC diventerebbe interessante se volessi che la stiffness fosse scelta *guardando avanti nel tempo* rispetto a un modello dinamico esplicito di:
- oggetto afferrato (massa, inerzia) tra i due end-effector,
- modello di contatto (rigidezza ambientale, eventuale slittamento),
- vincoli espliciti su forza massima, velocità massima, escursione di $K$ nel tempo.

In quel caso l'architettura tipica è **a cascata, non sostitutiva**:
- un MPC a frequenza più bassa (es. 20–50 Hz contro i tuoi probabili 200–500 Hz del QP) che risolve un problema di ottimizzazione con orizzonte $N$ passi, decidendo una traiettoria di $K_i$ (o di riferimento di forza/posizione) che minimizza un costo che include esplicitamente il termine di disallineamento $e_i \cdot f_i$ nell'orizzonte futuro, soggetto a vincoli su $\dot K$, $K_{\min}/K_{\max}$, forza massima;
- l'output del MPC (il $K_i$ per il prossimo blocco di tempo, o la sua traiettoria) viene passato come riferimento al livello QP istantaneo, che continua a girare come fa oggi.

Il costo di questa strada è concreto: ti serve un modello dinamico ragionevolmente accurato dell'interazione (altrimenti l'MPC ottimizza su un modello sbagliato e non guadagni nulla rispetto all'euristica), un risolutore QP/NLP aggiuntivo con orizzonte, tuning della matrice di costo e dei pesi, e gestione della cascata a due frequenze. Per il fenomeno che descrivi — un disallineamento istantaneo rilevabile da un semplice prodotto scalare — non c'è evidenza che tu abbia bisogno di guardare avanti nel tempo: il fenomeno è "adesso l'errore e la forza sono opposti", non "tra 200ms lo saranno". Un MPC qui aggiungerebbe complessità senza un guadagno chiaro.

**Raccomandazione**: implementa prima la legge gradiente+tanh (Sezione 2), valida in simulazione sul task di traslazione laterale che hai già, e tieni il MPC come upgrade path solo se osservi limiti specifici che l'euristica non risolve (es. instabilità quando la forza cambia molto più veloce della costante di tempo $\gamma$ che hai scelto, o necessità di rispettare vincoli hard che l'euristica non garantisce per costruzione).

---

## 4. Punti di attenzione e vincoli pratici

1. **Interazione con `hold()`.** Quando `leftImpedanceTask_->hold(true)` è attivo, la deformazione elastica è congelata: l'errore "visto" dal task non è più significativo per il calcolo di $s_i$ nello stesso modo. Decidi esplicitamente cosa fare in quello stato: o congeli anche l'adattamento di $K_i$ (mantieni l'ultimo valore, coerente con lo spirito di `hold`), oppure lo lasci correre ma sapendo che $e_i$ misurato durante hold non riflette il comportamento del braccio reale. Consiglio: congela $K_i$ quando `hold()==true`, così eviti che l'adattamento "prepari" un valore che diventa rilevante di colpo al rilascio dell'hold.

2. **$K_{\min}$ non troppo basso.** Un $K_{\min}$ troppo vicino a zero rende il braccio quasi in puro force-control sull'asse in questione: se l'inerzia dell'oggetto o un ritardo cinematico esiste (esattamente il problema che citi tu nel commento sopra `hold()`), rischi overshoot o distacco. Parti da un $K_{\min}$ che sia comunque una frazione ragionevole (es. 30–50%) del valore hardcodato attuale (50.0 sul sinistro), non da zero.

3. **Filtraggio della forza misurata.** `measuredWrench()` è tipicamente rumorosa (sensore F/T o stima da coppie giunto). Non alimentare $s_i$ con la forza grezza — riusa lo stesso stile di low-pass che hai già su `lambdaFiltered_lowpass_cut`.

4. **Segno e convenzione di $e_i$.** Verifica la convenzione esatta usata da `ImpedanceTask` per l'errore interno (target - measured, o measured - target) prima di calcolare $s_i$ a mano: se la inverti, il segno di $s_i$ si inverte e la legge fa l'opposto di quello che vuoi. Verificalo empiricamente in simulazione loggando $e_i$, $f_i$, $s_i$ insieme nel caso noto (traslazione laterale) dove sai già qual è il comportamento fisico atteso.

5. **Isolamento assi.** Applica l'adattamento solo alle componenti (indici 3:6 traslazionali, o anche solo l'asse laterale specifico) coinvolte nel fenomeno. Non toccare gli assi verticali dove hai già `wrenchGains` e la logica di force-tracking dell'`APPROACH_RAMP` / hold — mescolare i due meccanismi su uno stesso asse crea competizione difficile da debuggare.

---

## 5. Pseudocodice di integrazione

Struttura proposta, coerente con lo stile del tuo file (`DualArmControl.h` da estendere con i nuovi membri).

```cpp
// --- Nuovi membri in DualArmControl.h ---
// Eigen::Matrix<double,6,1> K_left_, K_right_;   // stato corrente, persistente tra i cicli
// double Kmin_ = 25.0, Kmax_ = 150.0;            // bound fisici, da tarare
// double alphaTanh_ = 2.0;                       // sharpness della mappa statica
// double gammaGain_ = 3.0;                       // 1/costante di tempo del filtro (rad/s equivalente)
// double sFiltered_left_ = 0.0, sFiltered_right_ = 0.0; // low-pass sull'allineamento

double DualArmControl::computeAlignment(const Eigen::Vector3d &e, const Eigen::Vector3d &f,
                                         const Eigen::Vector3d &axis)
{
    // proiezione sull'asse di interesse (es. asse x del grasp frame, world-frame)
    double e_p = e.dot(axis);
    double f_p = f.dot(axis);
    const double eps = 1e-3;
    return (e_p * f_p) / (std::abs(e_p) * std::abs(f_p) + eps);
    // nota: qui uso il prodotto di scalari con segno, non norme vettoriali,
    // perché stiamo già lavorando su una proiezione 1D
}

double DualArmControl::adaptStiffness(double K_current, double s_aligned, double dt)
{
    double Ktarget = Kmin_ + 0.5 * (Kmax_ - Kmin_) * (1.0 + std::tanh(alphaTanh_ * s_aligned));
    double Knext = K_current + gammaGain_ * dt * (Ktarget - K_current);
    return Knext;
}

// --- Dentro COOPERATIVE_MOTION, al posto dei valori hardcodati ---
if (!leftImpedanceTask_->hold())  // congela l'adattamento se in hold, vedi Sezione 4.1
{
    Eigen::Vector3d axis = x_0_objectCurrent_.rotation().row(0); // asse x del grasp frame in world
    Eigen::Vector3d e_left = (leftOffset_ * x_0_objectCurrent_).translation()
                              - robots().robot(leftRobotIndex_).bodyPosW(eeName_).translation();
    Eigen::Vector3d f_left = leftImpedanceTask_->measuredWrench().force();

    double s_left = computeAlignment(e_left, f_left, axis);
    sFiltered_left_ += 0.2 * (s_left - sFiltered_left_); // low-pass semplice, tau ~ dipende da 0.2/dt

    double Klat = adaptStiffness(K_left_(3), sFiltered_left_, timeStep); // indice 3 = traslazione x, esempio
    K_left_(3) = Klat; // eventualmente anche (4) se il fenomeno coinvolge più assi
}
// K_right_ analogo, oppure lasciato fisso/gestito con la sua legge simmetrica se serve

setImpedanceGains(K_left_, K_right_, gains.wrenchGains, 1);
```

Punti da adattare al tuo header reale: nomi esatti dei membri di `ImpedanceGains`/`setImpedanceGains`, convenzione di segno di `measuredWrench()`, e se `X_world_object`/`x_0_objectCurrent_` espone la rotazione nel modo che ho assunto (`.rotation().row(0)` per l'asse x, verifica in base alla convenzione riga/colonna usata altrove nel tuo `buildGraspFrame`, dove usi `R.col(0) = x`).

---

## 6. Piano di validazione consigliato

1. **Solo logging, guadagni ancora hardcodati.** Prima di toccare `setImpedanceGains`, aggiungi il log di $e_i$, $f_i$, $s_i$ nel task di traslazione laterale esistente. Verifica che il segno di $s_i$ corrisponda davvero al fenomeno fisico che descrivi (dovresti vedere $s_{\text{left}} < 0$ proprio nella fase in cui oggi compensi con `K=50` fisso).
2. **Sostituisci solo la mappa statica (tanh), senza filtro dinamico**, e osserva se compaiono discontinuità/jerk quando $s_i$ attraversa zero.
3. **Aggiungi il filtro del primo ordine ($\gamma$)** e tara la costante di tempo finché il jerk osservato al punto 2 sparisce, senza però rendere la risposta troppo lenta rispetto alla dinamica del task (la traiettoria quintica ha durata `gains.totalTrajectoryDuration_` — la costante di tempo di $K$ dovrebbe essere sensibilmente più corta di quella, altrimenti la stiffness insegue sempre in ritardo).
4. **Test di robustezza**: introduci rumore sintetico su `measuredWrench()` in simulazione e verifica che $K_i$ non oscilli in modo percepibile — se lo fa, aumenta il filtro su $s_i$ o riduci $\gamma$.
5. **Confronto quantitativo** con la versione hardcodata attuale: stessa traiettoria, stesso setup, confronta forza di picco, tempo di assestamento del grasp, e eventuali eventi di `hold()` release per crollo forza (Sezione "CASO A" nel tuo codice) — l'aspettativa è che la versione adattiva riduca la frequenza di quegli eventi rispetto al $K=50$ fisso.

---

## 7. Riferimenti utili (per approfondire, se ti serve la parte teorica più formale)

- Hogan, N., *Impedance Control: An Approach to Manipulation*, 1985 — fondamenti del controllo a impedenza.
- Ott, Albu-Schäffer et al. — lavori su variable/adaptive impedance control per bracci robotici a rigidezza variabile, base teorica per leggi $K(t)$ dipendenti dallo stato di interazione.
- Kronander & Billard — lavori su variable impedance learning, utile se in futuro vuoi sostituire la mappa tanh con una appresa da dati invece che progettata a mano.
- Caccavale et al. — controllo di forza interna/esterna per manipolazione dual-arm tramite grasp matrix, stesso formalismo che già usi in `buildGraspFrame`/`G_`/`Gpinv_`.

Questi ti servono solo se vuoi irrobustire teoricamente le scelte (es. dimostrare passività della legge scelta); per l'implementazione pratica dei prossimi giorni la Sezione 5 è il punto di partenza operativo.

Hai ragione, con la traiettoria pianificata a priori il problema si semplifica parecchio — non ti serve più il vettore errore generico, ti basta la direzione di moto nota analiticamente.

## 1. Come rilevi "stessa direzione, verso opposto"

Dato che conosci a priori x_0_objectStart_ e x_0_objectWaypoint1_, la direzione di moto è un vettore fisso e noto, non va stimato online:

$$ \hat{d}=\frac{∥targetPos−startPos∥}{targetPos−startPos}​ $$

(per traslazione lungo un solo asse, ma se la traiettoria ruota anche, $\hat d$
 va ricalcolato ad ogni istante come direzione della velocità desiderata istantanea — derivata di computeDesiredObjectPose(); per il caso lineare che hai ora è costante).

Il profilo quintico $s(t)$ è monotono crescente ($\dot s \ge 0$ sempre), quindi la velocità desiderata dell'end-effector ha sempre verso $+\hat d$
, mai $-\hat d$. Questo è il punto chiave: non devi confrontare due vettori generici (moto vs forza), il verso del moto è già noto e fisso, quindi ti basta un solo numero:

$$f_{\parallel,i}​(t)=f_i ​(t) \hat{d}$$

dove $f_i$​ è impedanceTask_i->measuredWrench().force().

- $ f_{\parallel,i} ​>0 $ → la forza spinge nello stesso verso del moto pianificato → il contatto "asseconda" il braccio → può restare rigido.
- $f_{\parallel,i} < 0$ → la forza spinge in verso opposto al moto pianificato, pur essendo sulla stessa direzione/asse → questo è esattamente il caso che descrivi: il braccio sta cercando di andare avanti lungo $\hat d$ ma il contatto lo tira indietro → deve diventare compliant.

Niente norme di errore, niente epsilon per divisioni — è solo il segno (e l'ampiezza) di uno scalare. Molto più robusto e più facile da debuggare via log rispetto alla cosine similarity generica che ti avevo proposto prima, perché qui il "moto" non è stimato da un errore rumoroso ma è un dato di pianificazione che già possiedi.

## 2. La politica di stiffness adattiva

Stessa struttura tanh + filtro che avevi visto, ma ora guidata da $f_{\parallel,i}$ invece che da una cosine similarity:
Mappa statica (tanh):
$$K_{target,i}​ = K_{min}​ + \frac{K_{max}​−K_{min}}{2}
​(1+tanh(\alpha \frac{f_{\parallel,i}}{f_{ref}}​​))$$

$f_{ref}$​ è una scala di normalizzazione usa qualcosa che hai già, tipo il tuo criticalForceThreshold (8.0 N) o contactThreshold, così $\alpha$ resta un numero puro facile da tarare (parti da $\alpha \approx 1-2$).

Dinamica (gradiente/filtro, per evitare salti):

$$K_i​[k+1]=K_i​[k]+\gamma \Delta t(K_{target,i}​−K_i​[k])$$

## 3. Integrazione nel tuo codice

```cpp
// direzione nota a priori, calcolata una volta in BUILD_GRASP
Eigen::Vector3d dHat = (x_0_objectWaypoint1_.translation() - x_0_objectStart_.translation()).normalized();

// dentro COOPERATIVE_MOTION, per ciascun braccio
Eigen::Vector3d fLeft = leftImpedanceTask_->measuredWrench().force();
double fParLeft = fLeft.dot(dHat);

// filtro leggero sulla forza proiettata, per non inseguire il rumore
fParLeftFiltered_ += 0.15 * (fParLeft - fParLeftFiltered_);

double Ktarget = Kmin_ + 0.5*(Kmax_-Kmin_) * (1.0 + std::tanh(alpha_ * fParLeftFiltered_ / criticalForceThreshold));
K_left_(3) += gamma_ * timeStep * (Ktarget - K_left_(3)); // asse traslazionale coinvolto
```

K_right_ puoi farlo simmetrico (stessa $\hat d$, stesso measuredWrench() del destro) — spesso troverai che è naturalmente quasi sempre $f_{\parallel,right} \approx f_{\parallel,left} $, per la reazione di afferraggio, quindi i due bracci finiranno per adattarsi in modo complementare senza che tu debba hardcodare chi è il "leader".