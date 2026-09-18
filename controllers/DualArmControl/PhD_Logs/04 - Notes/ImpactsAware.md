# Impact-Aware Manipulation: dalla teoria allo stato dell'arte, verso una soluzione applicata

---

## 1. Introduzione

### 1.1 Cos'è la "impact-aware manipulation"

In robotica manipolativa, la transizione dal moto libero (*free motion*) al contatto con l'ambiente o con un oggetto è tradizionalmente uno dei momenti più critici del controllo. Nella grande maggioranza delle applicazioni industriali e di ricerca, i robot vengono comandati ad avvicinarsi al punto di contatto a velocità pressoché nulla ("guarded motion"), in modo da minimizzare le forze impulsive scambiate al momento del contatto. Wang e Kheddar osservano che, a differenza degli esseri umani, i robot sono tenuti a muoversi a velocità quasi nulla in prossimità del contatto proprio per questo motivo, il che riduce sensibilmente la velocità di esecuzione dei compiti che comportano interazione fisica.

La *impact-aware manipulation* è il filone di ricerca che affronta il problema opposto: progettare pianificatori e controllori che non trattino l'impatto come un evento da evitare a ogni costo, ma come una fase dinamica del compito, modellata esplicitamente, prevista in anticipo e gestita in modo da renderla sicura, ripetibile e, quando utile, persino sfruttabile (ad esempio per compiti di tipo "hit-and-push" o afferraggio in movimento).

### 1.2 Perché ce ne si occupa

Le motivazioni principali che la letteratura individua per lo studio dell'impact-aware control sono essenzialmente tre:

1. **Sicurezza hardware.** Un impatto non modellato produce discontinuità (jump) nelle velocità di giunto e negli sforzi articolari che possono superare i limiti hardware del manipolatore, danneggiando riduttori, sensori di coppia o l'end-effector stesso.
2. **Prestazioni e velocità del compito.** L'approccio "guarded/slow motion" è robusto ma penalizzante in termini di tempo ciclo: rallentare artificialmente ogni avvicinamento a un contatto è inaccettabile in contesti industriali o collaborativi ad alto throughput.
3. **Qualità del controllo post-contatto.** Se il controllore reagisce solo dopo che l'impatto è avvenuto (tipicamente tramite soglia di forza), il picco di forza si è già scaricato sulla struttura prima che il loop di controllo a forza/impedenza abbia il tempo di intervenire, generando transitori di forza elevati e potenzialmente instabilità di contatto.

### 1.3 Struttura del documento

Il documento procede come segue: la Sezione 2 introduce il modello fisico dell'impatto in ambito robotico (quantità di moto operazionale, matrice di inerzia operazionale, impulso di contatto). La Sezione 3 presenta lo stato dell'arte, organizzato per famiglie di approcci. La Sezione 4 tratta la fase successiva all'impatto, ovvero la manipolazione cooperativa/con forza interna. La Sezione 5, **solo alla fine**, applica quanto discusso al caso specifico, a partire dalla pipeline attualmente in uso.

---

## 2. Fondamenti teorici: la fisica dell'impatto in manipolazione robotica

### 2.1 Dinamica nello spazio operazionale

La formulazione dello spazio operazionale (*Operational Space Formulation*, OSF), introdotta da Khatib, descrive la dinamica dell'end-effector di un manipolatore in termini di grandezze cartesiane invece che di giunto. Definendo J come lo Jacobiano dell'end-effector e M la matrice di inerzia in spazio dei giunti, la dinamica dell'end-effector è governata dalla matrice di inerzia operazionale (od *Operational-Space Inertia Matrix*, OSIM):

$$\Lambda = (J M^{-1} J^T)^{-1}$$

Λ rappresenta l'inerzia "percepita" al punto di contatto, cioè quanta massa il resto della catena cinematica riflette in quel punto. Questa quantità è centrale in tutta la letteratura sull'impatto robotico, poiché è il parametro che lega direttamente velocità di avvicinamento e severità dell'impatto.

### 2.2 Energia cinetica e impulso di contatto

Prima dell'impatto, l'end-effector possiede una velocità operazionale v e quindi un'energia cinetica

$$E = \frac{1}{2} v^T \Lambda v$$

Durante l'impatto, modellato come evento (quasi) istantaneo secondo la meccanica non regolare (*nonsmooth mechanics* — si vedano i testi di riferimento di Brogliato e di Glocker), la velocità dell'end-effector subisce un salto discontinuo da v_pre a v_post. L'impulso di contatto scambiato è

$$P = \Lambda (v_{post} - v_{pre})$$

Assumendo che l'impatto duri un intervallo di tempo Δt (fisicamente non nullo, ma molto breve), la forza media di contatto risultante è

$$F_{avg} = \frac{P}{\Delta t}$$

Questa relazione, per quanto semplice, spiega un fatto empirico ben noto: **a parità di velocità di impatto, una maggiore inerzia operazionale Λ produce impulsi e forze di picco proporzionalmente più elevati**. È esattamente questo il meccanismo fisico dietro ai picchi di forza da impatto osservati sperimentalmente (nell'ordine di decine di N per impatti anche moderati), ed è il motivo per cui gran parte della letteratura recente lavora sia sulla riduzione della velocità normale di contatto sia sulla riduzione/gestione dell'inerzia operazionale riflessa nella direzione di impatto.

### 2.3 Predizione post-impatto: un problema tuttora aperto

Un filone specifico di ricerca, distinto ma complementare al controllo vero e proprio, riguarda la predizione accurata di v_post a partire da v_pre. Wang, Dehio e Kheddar mostrano che gli approcci "standard" basati sull'inversione diretta della matrice di inerzia in spazio dei giunti sono affetti da errori di predizione considerevoli, e propongono una formulazione in spazio operazionale che, modellando il manipolatore controllato in posizione ad alto guadagno come un corpo rigido composito, riduce l'errore medio di predizione di circa l'82% rispetto all'approccio comune, verificato su 250 prove sperimentali con un manipolatore Panda. Analogamente, van Steen e colleghi hanno recentemente quantificato lo scarto fra simulazione fisica e dati reali nella mappa ante-impatto/post-impatto, ottenendo un errore medio del 3.1% fra velocità post-impatto identificata sperimentalmente e quella predetta da simulazione, utilizzando un framework di validazione basato sul *reference spreading control* (si veda §3.3).

Questo sotto-filone è rilevante perché **tutti** gli approcci di controllo impact-aware descritti di seguito, in un modo o nell'altro, presuppongono un modello (più o meno accurato) dell'impatto stesso: la qualità del modello di impatto è quindi un limite pratico trasversale a tutta la letteratura.

---

## 3. Stato dell'arte: strategie di controllo per la transizione al contatto

La letteratura può essere organizzata in quattro famiglie di approcci, in ordine crescente di sofisticazione del modello di impatto utilizzato.

### 3.1 Approccio 1 — Guarded / slow motion

È l'approccio industriale classico: il robot rallenta progressivamente man mano che si avvicina al punto di contatto atteso, così da minimizzare v_pre e quindi, per la relazione P = Λ(v_post − v_pre), l'impulso scambiato. Vantaggi: robustezza intrinseca, semplicità implementativa, nessun bisogno di un modello di impatto. Limiti: richiede una stima affidabile della distanza residua dal contatto (visione, prossimità, o un modello geometrico accurato dell'ambiente), e penalizza pesantemente il tempo ciclo — è proprio il comportamento che la letteratura su impact-aware manipulation cerca di superare.

### 3.2 Approccio 2 — Compliance / impedance control

Il controllo a impedenza, introdotto nella formulazione classica in tre parti da Hogan, non comanda direttamente una forza o una posizione, ma una relazione dinamica desiderata (tipicamente massa-molla-smorzatore) fra l'errore di posizione dell'end-effector e la forza di interazione scambiata con l'ambiente. Riducendo i guadagni di rigidezza K (e in alcuni casi anche di smorzamento) nella fase immediatamente precedente al contatto atteso, l'end-effector diventa "cedevole" e può assorbire parte dell'energia cinetica dell'impatto per deformazione elastica virtuale, piuttosto che scaricarla istantaneamente come forza di reazione.

Il tema della stabilità di contatto in questo contesto è stato affrontato sin dai lavori originari di Hogan sulla stabilità dei manipolatori nei compiti di contatto, e successivamente formalizzato in termini di passività da Colgate e Hogan, che hanno mostrato come l'instabilità di contatto in presenza di ambienti rigidi sia legata alla non-passività dell'impedenza realizzata dal controllore quando questo opera in catena con ritardi, quantizzazione e banda finita del loop di controllo. Vantaggi: non richiede necessariamente un modello di impatto esplicito, è relativamente semplice da integrare in un controllore a coppia. Limiti: la riduzione di rigidezza non elimina l'impulso, lo ridistribuisce nel tempo; se l'ambiente è molto rigido o la velocità di impatto non trascurabile, il beneficio è limitato; inoltre una rigidezza troppo bassa compromette la precisione di posizionamento nella fase di avvicinamento.

Una variante rilevante è quella dei controllori adattivi con limitazione di forza durante la transizione da non-contatto a contatto, come nel lavoro di Dixon e colleghi sul controllo adattivo con limitazione della forza per sistemi robotici in transizione non-contatto/contatto, che affronta esplicitamente il problema di bound sulla forza durante la fase di stabilizzazione post-contatto.

### 3.3 Approccio 3 — Reference spreading control

Un filone distinto, sviluppato principalmente dal gruppo di Saccon, van de Wouw e collaboratori (TU Eindhoven / TU Delft), affronta il problema da un'angolazione di *controllo del tracking* piuttosto che di ottimizzazione istantanea. L'idea centrale del *reference spreading* è quella di definire due riferimenti di moto — uno ante-impatto e uno post-impatto — che si sovrappongono parzialmente in un intorno temporale/spaziale dell'impatto atteso, collegati fra loro tramite una mappa di impatto rigido. In questo modo l'errore di tracking non presenta il picco artificioso che si avrebbe tracciando un riferimento discontinuo con un controllore continuo, e si evitano i picchi di sforzo di controllo tipicamente indotti dall'impatto.

Questo framework è stato esteso in diverse direzioni: al caso di afferraggio bimanuale con impatti pianificati simultaneamente, alla versione "time-invariant" per l'utilizzo intenzionale di impatti (ad es. compiti di tipo hit-and-push, validati sperimentalmente su 600 prove robotiche), e più recentemente utilizzato come strumento di validazione sperimentale della mappa di impatto rigido stessa. Un'estensione robusta per robot a giunti torque-controlled con elasticità di giunto (flexible-joint) è stata proposta da Arias e colleghi. Vantaggi: elimina i picchi di tracking error tipici della transizione discontinua, buona validazione sperimentale su piattaforme reali (Panda, sistemi bimanuali). Limiti: richiede la conoscenza (o pianificazione) dell'istante e della locazione dell'impatto con ragionevole anticipo, ed è concettualmente più legato al tracking control che alla generazione autonoma della traiettoria di avvicinamento.

### 3.4 Approccio 4 — Impact-aware optimization basata su QP

Questa è la famiglia di approcci più direttamente rilevante per il problema descritto, e comprende sia i lavori di Wang e Kheddar (e successivi, con Dehio e Tanguy) sia quelli di van Steen e colleghi che formulano esplicitamente l'impatto come vincolo o termine di costo dentro un controllore basato su programmazione quadratica (QP).

**Idea di fondo.** Invece di attendere che il sensore di forza rilevi il contatto già avvenuto, il controllore, ad ogni ciclo di controllo, **assume che l'impatto possa verificarsi al passo successivo** e ottimizza il comando di controllo tenendo conto esplicitamente delle conseguenze dinamiche di questa eventualità. Wang, Dehio, Tanguy e Kheddar descrivono questo meccanismo come una sorta di "one-step preview": in prossimità del punto di contatto target, ad ogni ciclo si assume che l'impatto avvenga al ciclo successivo, il che rende il controllore robusto rispetto a incertezze sul tempo e sulla locazione esatta dell'impatto.

**Formulazione.** Il controllore QP a task-space viene arricchito con vincoli e/o termini di costo aggiuntivi legati a grandezze correlate alla severità dell'impatto:

- **quantità di moto operazionale** Λv, penalizzata o vincolata in norma;
- **impulso predetto** Λ(v_post − v_pre), stimato a partire da un modello di impatto (rigido o con coefficiente di restituzione);
- **velocità normale al contatto** v_n, la componente di velocità nella direzione della normale di contatto attesa, che è la grandezza più direttamente responsabile della severità dell'impatto secondo la relazione P = Λ(v_post − v_pre).

Wang e Kheddar, nella loro prima formulazione, introducono esplicitamente il modello discreto della dinamica di impatto dentro il controllore QP, in modo da generare moti robusti ai salti di stato indotti dall'impatto sia sulle velocità di giunto sia sulle coppie. Il lavoro successivo (*impact-aware task-space quadratic-programming control*) estende l'approccio a impatti frizionali tridimensionali, riformulando i vincoli standard del QP per tenere conto dei salti di stato e introducendo (i) vincoli legati ai limiti hardware del robot rispetto all'impatto, e (ii) un insieme ammissibile calcolato analiticamente (poliedro) che vincola gli stati critici post-impatto; il tutto validato sperimentalmente su un manipolatore Panda e su compiti di afferraggio rapido con l'umanoide HRP-4.

Vantaggi di questa famiglia di approcci: (i) trasforma l'impatto da evento subito a evento *pianificato e controllato*, permettendo velocità di avvicinamento non nulle; (ii) è naturalmente compatibile con controllori QP multi-task già esistenti (basta aggiungere vincoli/costi, senza riprogettare l'intera architettura); (iii) consente sia un uso "difensivo" (minimizzare la severità dell'impatto) sia un uso "offensivo" (sfruttare l'impatto per compiti dinamici come lancio/spinta di oggetti). Limiti principali: dipende dalla qualità del modello di impatto (si veda §2.3) e dalla stima real-time di Λ e della normale di contatto attesa; introduce complessità computazionale aggiuntiva nel solver QP che deve girare in tempo reale.

### 3.5 Sintesi comparativa

| Approccio | Modello di impatto richiesto | Velocità di avvicinamento consentita | Complessità implementativa | Riferimenti chiave |
|---|---|---|---|---|
| Guarded/slow motion | Nessuno | Quasi nulla | Bassa | pratica industriale consolidata |
| Impedance/compliance | Nessuno (o approssimato) | Bassa-media | Media | Hogan 1985; Colgate & Hogan 1989 |
| Reference spreading | Mappa di impatto rigido | Media | Medio-alta | van Steen et al. 2022–2024 |
| QP impact-aware optimization | Modello di impatto (rigido/impulsivo) esplicito nel QP | Media-alta | Alta | Wang & Kheddar 2019; Wang et al. 2023 |

---

## 4. La fase successiva: dal contatto stabile alla manipolazione cooperativa

Una volta che il contatto è stato stabilito (e stabilizzato, con uno degli approcci della Sezione 3), nel caso di manipolazione cooperativa multi-arto la letteratura classica descrive la transizione al controllo collaborativo tramite la **matrice di presa** (*grasp matrix*), che mappa le forze/coppie applicate dai singoli end-effector alla risultante di forza/coppia agente sull'oggetto manipolato.

Caccavale e Uchiyama, nel capitolo di riferimento dello Springer Handbook of Robotics, ripercorrono la storia della manipolazione cooperativa e formalizzano il modello dinamico di un oggetto rigido afferrato da più bracci tramite la matrice di presa, distinguendo le forze che contribuiscono al moto dell'oggetto da quelle **interne** al sistema (forze di serraggio/precarico che si scaricano tra i bracci attraverso l'oggetto senza produrne il moto). Il concetto di forza interna, e la sua rappresentazione tramite il modello del "virtual linkage", è stato introdotto da Williams e Khatib per modellare in modo compatto le forze interne nella presa multi-contatto.

Su questa base si sono sviluppati diversi schemi di controllo per la fase cooperativa:

- **controllo di impedenza a 6 gradi di libertà per manipolatori cooperativi**, proposto da Caccavale, Chiacchio, Marino e Villani, che regola simultaneamente il comportamento dinamico dell'oggetto manipolato e le forze interne di serraggio;
- **controllo di impedenza basato su forze interne ed esterne**, proposto da Heck, Kostić e colleghi, che distingue esplicitamente l'anello di regolazione delle forze interne (per evitare danni all'oggetto o al robot da un serraggio eccessivo) da quello delle forze esterne/di moto;
- **controllo di impedenza cooperativo per bracci con giunti elastici**, che estende l'approccio a manipolatori con cedevolezza di giunto, rilevante quando la catena cinematica non è idealmente rigida.

Va sottolineato che **questa fase è concettualmente successiva e disaccoppiata** da quella di impact-aware control descritta nella Sezione 3: la letteratura sulla manipolazione cooperativa assume tipicamente che il contatto sia già stabilito con velocità relativa trascurabile, e si concentra sulla regolazione di forze interne/esterne a regime. È proprio nell'interfaccia fra queste due fasi — impatto e stabilizzazione del contatto da un lato, costruzione della matrice di presa e passaggio al controllo cooperativo dall'altro — che si colloca il problema descritto nella Sezione 5.

---

## 5. Applicazione al caso specifico

### 5.1 Pipeline attuale e diagnosi del problema, alla luce della teoria

La pipeline attualmente in uso è:

```
Reaching → Contatto rilevato da soglia di forza → Costruzione Grasp Matrix → Switch a controllo di forza cooperativo
```

Alla luce di quanto discusso nelle Sezioni 2–4, il problema puó essere riformulato in termini precisi: il rilevamento del contatto tramite soglia di forza è, per costruzione, un evento **a posteriori** — il sensore misura F solo dopo che l'impulso P = Λ(v_post − v_pre) si è già scaricato sulla struttura. Poiché nella fase di reaching non vi è alcuna riduzione né dell'inerzia operazionale riflessa Λ né della velocità di avvicinamento v_pre, e non vi è alcun modello dell'impatto imminente, il sistema si comporta come il caso peggiore fra quelli discussi in letteratura: nessuna delle quattro famiglie di approcci (§3.1–3.4) è di fatto attiva durante l'avvicinamento. I picchi di 60–80 N osservati sono coerenti con questa lettura: sono l'impulso "pieno", non attenuato né da riduzione di velocità, né da cedevolezza, né da un modello predittivo.

### 5.2 Verso una pipeline impact-aware: proposta

Combinando gli approcci più maturi e meglio validati sperimentalmente in letteratura (§3.2 e §3.4), una possibile evoluzione della pipeline introduce una fase esplicita di **transizione d'impatto** fra il reaching e la costruzione della grasp matrix:

```
Reach
 │
 ├── Cartesian tracking
 ├── Contact prediction (stima distanza/tempo al contatto atteso)
 ▼
Impact phase
 │
 ├── Riduzione di impedenza (K, D) nella direzione normale di contatto attesa
 ├── Vincoli/costi impact-aware nel QP (limite su v_n, su Λ(v_post-v_pre))
 ├── Limitazione della velocità normale di contatto
 ▼
Stable contact
 │
 ├── Costruzione Grasp Matrix
 ├── Freeze del frame di contatto
 ▼
Collaborative manipulation
 │
 ├── Controllo forze interne
 └── Manipolazione cooperativa dell'oggetto
```

Nel merito delle scelte progettuali, in ordine di complessità/beneficio crescente:

1. **Predizione del contatto.** È il prerequisito comune a tutti gli approcci più sofisticati della Sezione 3 (eccetto il guarded motion puro): serve una stima, anche approssimata, della distanza/tempo residuo al contatto atteso — da modello geometrico del compito, da visione, o da un semplice threshold sulla posizione cartesiana rispetto a una superficie nota. Senza questo elemento, nessuna delle strategie 3.2–3.4 è applicabile in anticipo rispetto al contatto.
2. **Riduzione di impedenza pre-contatto (§3.2).** È l'intervento a più basso costo implementativo, se il robot è già controllato in coppia: riducendo K (e opportunamente D, per mantenere lo smorzamento critico secondo l'analisi di stabilità di Colgate e Hogan) nella finestra temporale immediatamente precedente al contatto atteso, si attenua l'impulso trasmesso senza richiedere un modello di impatto esplicito.
3. **Vincoli impact-aware nel QP (§3.4).** È l'intervento più efficace secondo la letteratura più recente, ma richiede (i) una stima real-time di Λ nella direzione di contatto, (ii) un modello dell'impatto (anche il semplice modello rigido con coefficiente di restituzione è un buon punto di partenza, secondo quanto validato da van Steen et al. con un errore medio del 3.1%), e (iii) l'integrazione di questi vincoli nel solver QP esistente. Se il controllore corrente è già basato su QP task-space, questo è l'intervento con il miglior rapporto beneficio/intrusività architetturale, in linea con quanto riportato da Wang et al. per compiti di afferraggio rapido.
4. **Freeze del contact frame e costruzione della grasp matrix** rimangono, secondo la letteratura sulla manipolazione cooperativa (§4), correttamente collocati **dopo** la stabilizzazione del contatto: è solo a questo punto che le ipotesi di velocità relativa trascurabile, su cui si basano i controllori a forza interna di Caccavale et al. e di Heck et al., risultano valide.

### 5.3 Considerazioni finali

Il punto centrale che emerge dallo stato dell'arte è che il rilevamento a soglia di forza non è di per sé errato come meccanismo di **conferma** del contatto stabile, ma è strutturalmente inadeguato come unico segnale per **avviare** la gestione del contatto: la letteratura converge nel trattare l'impatto come una fase dinamica a sé, da prevedere e modellare esplicitamente (con gradi di sofisticazione crescente da §3.1 a §3.4), e solo successivamente da consegnare al livello di controllo cooperativo che presuppone un contatto già stabilizzato.

---

## Riferimenti

- Khatib, O. (1987). *A unified approach for motion and force control of robot manipulators: The operational space formulation.* IEEE Journal of Robotics and Automation, 3(1), 43–53.
- Hogan, N. (1985). *Impedance Control: An Approach to Manipulation, Parts I–III.* Journal of Dynamic Systems, Measurement, and Control, 107, 1–24.
- Colgate, J. E., Hogan, N. (1989). *An analysis of contact instability in terms of passive physical equivalents.* IEEE International Conference on Robotics and Automation.
- Brogliato, B. (2016). *Nonsmooth Mechanics*, 3rd ed. Springer.
- Glocker, C. (2006). *An Introduction to Impacts.* In Nonsmooth Mechanics of Solids, CISM Courses and Lectures, vol. 485. Springer.
- Wang, Y., Kheddar, A. (2019). *Impact-friendly robust control design with task-space quadratic optimization.* Robotics: Science and Systems XV.
- Wang, Y., Dehio, N., Tanguy, A., Kheddar, A. (2023). *Impact-aware task-space quadratic-programming control.* The International Journal of Robotics Research, 42(14), 1265–1282.
- Wang, Y., Dehio, N., Kheddar, A. *Predicting Impact-Induced Joint Velocity Jumps on Kinematic-Controlled Manipulator.* arXiv:2202.12646.
- van Steen, J., van den Brandt, G., van de Wouw, N., Kober, J., Saccon, A. (2024). *Quadratic Programming-based Reference Spreading Control for Dual-Arm Robotic Manipulation with Planned Simultaneous Impacts.* IEEE Transactions on Robotics, 40, 3341–3355.
- van Steen, J., Coşgun, A., van de Wouw, N., Saccon, A. (2023). *Dual Arm Impact-Aware Grasping through Time-Invariant Reference Spreading Control.* IFAC-PapersOnLine, 56(2), 1009–1016.
- van Steen, J., et al. (2024). *Impact-Aware Control using Time-Invariant Reference Spreading.* arXiv:2411.09870.
- van Steen, J., Stokbroekx, D., van de Wouw, N., Saccon, A. (2024). *Impact-Aware Robotic Manipulation: Quantifying the Sim-To-Real Gap for Velocity Jumps.* arXiv:2411.06319.
- Arias, C. A. R., Weekers, W., Morganti, M., Padois, V., Saccon, A. (2024). *Refined Post-Impact Velocity Prediction for Torque-Controlled Flexible-Joint Robots.* IEEE Robotics and Automation Letters, 9(4), 3267–3274.
- Dixon, W. E., et al. *A force limiting adaptive controller for a robotic system undergoing a noncontact-to-contact transition.* IEEE Transactions on Control Systems Technology.
- Caccavale, F., Uchiyama, M. (2016). *Cooperative Manipulation.* In Springer Handbook of Robotics, 2nd ed., Springer.
- Caccavale, F., Chiacchio, P., Marino, A., Villani, L. (2008). *Six-DOF impedance control of dual-arm cooperative manipulators.* IEEE/ASME Transactions on Mechatronics, 13(5), 576–586.
- Williams, D., Khatib, O. (1993). *The virtual linkage: a model for internal forces in multi-grasp manipulation.* IEEE International Conference on Robotics and Automation.
- Heck, D., Kostić, D., Denasi, A., et al. *Internal and external force-based impedance control for cooperative manipulation.*

*Nota: i riferimenti sopra elencati sono stati reperiti su Google Scholar / arXiv al momento della stesura di questo documento (luglio 2026). Si raccomanda di consultare le versioni integrali per i dettagli implementativi (formulazione esatta dei vincoli QP, guadagni di impedenza, parametri sperimentali).*






# 15-07-2026
la mia idea e di fare un impatto a velocita diversa da 0, che dissipa tutta la forza, ora se alzo parametri i robot "rimbalzano" come un urto elastico, voglio completamente anelastico, indipendentemente dalla velocita. mi immaggino dipo convey packges that we do not know the velocity, 
in this scenario will we have stationary packages so i limited the velocity

## Perché rimbalza
Il tuo schema attuale (ramp di forza in targetWrench → freeze della pose quando la soglia scatta) è, dal punto di vista dinamico, un contatto a molla: quando targetPose viene bloccata sulla posa corrente, ma l'end-effector ha ancora velocità residua non nulla in quell'istante, il task di impedenza la interpreta come un errore di posizione e reagisce con una forza di richiamo proporzionale a K. Se K è alto, quella forza di richiamo rispedisce indietro l'end-effector — è esattamente un urto elastico (coefficiente di restituzione e>0), non per via del sensore ma per via del comportamento a molla del controllore stesso. È lo stesso fenomeno descritto da Hogan e più tardi formalizzato da Colgate & Hogan in termini di stabilità/passività del contatto: rigidezza alta = comportamento vicino all'elastico, indipendentemente dal fatto che tu lo voglia o no.

## La soluzione più diretta: rapporto smorzamento/rigidezza, non solo K
Volpe e Khosla, in un lavoro specificamente dedicato a questo problema (A theoretical and experimental investigation of impact control for manipulators), mostrano che un contatto rigido-su-rigido senza rimbalzo si ottiene non abbassando la rigidezza in sé, ma agendo sul rapporto tra massa/inerzia apparente del controllore e quella dell'ambiente: con guadagni di forza proporzionali negativi, o un rapporto di massa impedenza-ambiente minore di 1, si ottiene un impatto stabile senza rimbalzo. Tradotto nel tuo codice: il problema non è "che K uso", ma il fatto che stai usando D = 2*sqrt(K) (smorzamento critico) — che dà una risposta oscillante-al-limite in teoria, ma basta un minimo errore di modello o ritardo del loop reale perché scivoli in sotto-smorzato → rimbalzo. Prova a sovrasmorzare deliberatamente nel momento del freeze:

## Se vuoi davvero "e = 0" garantito, indipendentemente dalla velocità residua

Qui entra in gioco un'idea diversa dalla tua pipeline attuale, mutuata dal reference spreading di van Steen: invece di lasciare che il coefficiente di restituzione emerga (bene o male) dai guadagni K/D, lo imponi esplicitamente nel riferimento. Quando il contatto viene rilevato, invece di congelare la posa corrente dell'end-effector (che può includere un piccolo overshoot), costruisci il riferimento post-contatto assumendo v_post = 0 per costruzione — cioè il target diventa "resta fermo alla posizione di primo contatto", e lasci che sia il controllore (con D sovrasmorzato) a portarcelo senza oscillare, invece di dargli un target che "vibra" se il freeze avviene qualche ciclo in ritardo rispetto al vero istante di contatto. È una differenza sottile ma è esattamente il punto del framework: non stai più sperando che i guadagni si comportino bene, stai dicendo esplicitamente al controllore "questo è un urto plastico" a livello di riferimento, non di tuning.

## Riferimenti aggiuntivi rilevanti

- Volpe, R., Khosla, P. (1993). A theoretical and experimental investigation of impact control for manipulators. The International Journal of Robotics Research, 12(4), 351–365.
- Colgate, J. E., Hogan, N. (1989). An analysis of contact instability in terms of passive physical equivalents. IEEE ICRA.
- Lange, F., Hirzinger, G. (o simile). Impact modeling and control for industrial manipulators — studio sperimentale su superficie rigida (granito) che mostra contatto senza rimbalzo tramite controllo integrale nella fase di transizione.

## e $ \leq $ 0
Se ζ = D/(2√(MK)) ≥ 1 (aperiodico/overdamped), questa risposta non oscilla mai: x cresce, raggiunge un massimo (penetrazione massima), poi decade monotonicamente a zero senza mai invertire di segno la velocità oltre l'istante iniziale. Questo è, matematicamente, e = 0 (o e<0 se strapotenzi lo smorzamento) per costruzione del modello, non per tuning empirico — è esattamente la condizione che stavi cercando.

## Il ruolo del mass ratio (Volpe & Khosla)
Qui entra il secondo pezzo, quello che avevi chiesto: il risultato di Volpe & Khosla dice che un rapporto massa virtuale/inerzia riflessa del braccio minore di 1 dà una risposta di impatto senza rimbalzo anche a fronte di incertezze di modello — è la condizione di robustezza in più rispetto al solo ζ≥1 su un modello ideale. Concretamente:

// Rapporto massa target vs inerzia riflessa del braccio: < 1 = "leggero", reagisce prima che accumuli energia
gains.massImpactGains << 0.3, 0.3, 0.3, 0.3, 0.3, 0.3; // invece di Constant(1)



# inerzia riflessa
In robotica e meccatronica, l'inerzia riflessa ($m_{\text{refl}}$ o $I_{\text{refl}}$) è l'inerzia equivalente che l'ambiente (o un osservatore esterno) "percepisce" quando spinge o interagisce con l'organo terminale (end-effector) del robot.
Essa non dipende solo dalla massa fisica dei link, ma è fortemente influenzata dai rapporti di riduzione dei motori.Se consideriamo un singolo giunto con un motore di inerzia rotorica $I_m$ collegato a un riduttore con rapporto di riduzione $N$ (dove $N > 1$, ad esempio $100:1$):

## L'Analogia Intuitiva: Il Cambio dell'Auto
Immagina di spingere a mano un'automobile spenta in folle. L'auto si muove (faticosamente), perché stai opponendo forza solo alla sua massa reale.
Ora immagina di fare la stessa cosa, ma con l'auto che ha la prima marcia inserita.
Se provi a spingerla, l'auto ti sembrerà pesante come un palazzo, quasi impossibile da muovere.

- La massa fisica dell'auto è rimasta esattamente la stessa di prima.

- Cos'è cambiato? Per far fare un millimetro alle ruote dell'auto, il motore (tramite la marcia corta) deve compiere tantissimi giri velocemente. Tu, da fuori, non stai solo accelerando la massa dell'auto, ma stai costringendo il motore a girare vorticosamente.

L'inerzia del motore, "moltiplicata" dal cambio e proiettata sulle ruote, è l'inerzia riflessa. Dall'esterno, la percepisci come un muro di mattoni.

## Come si calcola (Il concetto matematico)

Nei robot, i motori elettrici girano molto velocemente ma hanno poca coppia, quindi si usano quasi sempre dei riduttori di velocità (es. Harmonic Drive, riduttori epicicloidali) con un rapporto di riduzione $N$ (es. $N = 100$).L'inerzia totale che "sente" l'ambiente quando urta l'estremità del robot è data da:

$$m_{\text{riflessa}} = m_{\text{link}} + N^2 \cdot m_{\text{motore}}$$

Nota il quadrato ($N^2$): Se il motore ha un'inerzia minuscola (es. $0.001 \text{ kg}\cdot\text{m}^2$) ma il riduttore è $100:1$, l'inerzia del motore riflessa sul giunto diventa $100^2 \times 0.001 = 10 \text{ kg}\cdot\text{m}^2$. L'effetto del motore è amplificato di 10.000 volte!

## Come la "capisce" il robot durante un impatto?
Perché Volpe e Khosla insistono tanto su questo punto? Perché l'inerzia riflessa divide il comportamento del robot in due fasi temporali distinte quando tocca una superficie:
### Fase 1: L'Impatto Puro (I primi millisecondi)
Quando l'end-effector urta un oggetto, la corrente nei motori non cambia istantaneamente (c'è il ritardo del computer, dei sensori e dell'induttanza dei motori). Per i primi millisecondi, il robot è un "pezzo di ferro passivo".
In questo istante, l'ambiente vede solo l'inerzia riflessa. Se questa è altissima (a causa di riduttori elevati), l'impatto genererà una forza enorme (una martellata), che farà rimbalzare indietro il robot prima ancora che il software possa rendersene conto.
### Fase 2: La Risposta del Controllo (Dopo l'impatto)
Solo dopo qualche millisecondo il computer legge il sensore di forza e dice al motore di alleggerirsi. Qui entra in gioco la massa virtuale ($m_{\text{virtuale}}$).Ecco perché, come dicevi prima, se tu imposti una massa virtuale più piccola dell'inerzia riflessa ($m_{\text{virtuale}} < m_{\text{riflessa}}$), stai dicendo al controllore di essere estremamente "morbido" e reattivo per compensare quella tremenda "martellata" passiva iniziale dovuta all'inerzia riflessa, evitando il rimbalzo.



# 22 LUGLIO

cambiare architettura usando parte delle cose gia fatte ma inserendo un mpc, parallelo

## STRUTTURA

- **stato:** $$ [z, \dot{z}] $$ along the contact normal
 - maybe augmented with the forces

- **Pre-contact phase model:** simple double integrator, $$ \ddot{z} = u $$ ($$u = $$ commanded normal acceleration, bounded by your torque/velocity limits).

- **Terminal constraint (this is where last message's impulse model plugs in):** at the horizon step where predicted z crosses the contact surface, constrain the predicted contact velocity $v_n$ so that F_peak_predicted($v_n$, $m_reflected_z$, $e_max$) $\leq$ $F_max$ — i.e., the restitution/impulse formula from before becomes a terminal inequality constraint in the MPC, exactly like the UAV paper's restitution-embedded contact dynamics.

- **Cost:** track your nominal approach trajectory, penalize deviation and control effort, small terminal weight pulling $$v_n$$ toward a target contact speed (not necessarily zero — you may want moderate impact velocity for a fast grasp).

- **Output:** feed the MPC's planned ż (or spd(5)) into your existing boundSpeed() each cycle — no change needed to your mc_rtc integration, just where the reference comes from.

