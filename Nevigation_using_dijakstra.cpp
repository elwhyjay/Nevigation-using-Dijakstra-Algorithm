/*
Dijkstra Algorithm을 이용한 네비게이션 프로그램 구현 과제

*/
#include <iostream>
#include <algorithm>
#include <queue>
#include <stack>
#include <math.h>
#include <string>
#include <climits>
#include <bitset>
#include <functional>
using namespace std;
struct city { //vertex의 정보를 저장할 구조체
	int cityNum; //vertex의 지역번호 100,000 ~ 999,999
	string cityName; //vertex의 지역명
	vector< pair<int, int> > neighbor; //(neighbor, dist) 인접한 vertex의 지역번호 와 edge weight를 저장
	int isFlood; // 침수여부
};
#define over 1000000  //거리가 10^6 초과할시 경로가없음으로 간주
vector<int> visit; // vertex가 tree에 들어갔는지 검사하는 배열

				   /*arr[1000000]으로 인접리스트를 만들면 큰차이는 없겠지만 vertex수만큼의 리스트를 만드는것보다
				   비효율적이므로 지역번호에 대한 1:1 해싱배열을 만들어줌 중복된 지역번호가없음으로 collision은 고려하지않음 */
vector<int> arr(1000000); //list와 1:1 correspondence, hashing
vector<city> list; //vertex들의 인접리스트						  

				   /*Dijkstra Algorithm*/
int dijkstra(city A, city B, vector<int> &path, int &lastDist) { // 매개변수로 출발,도착 지역, vertex의 수를받음
	priority_queue< pair<int, int>, vector< pair <int, int> >, greater< pair<int, int> > > Minpq; // fringe vertex를 관리할 min heap(dist,cit number)
	visit.assign(list.size(), 0); // tree에 vertex도 없음
	vector<pair<int, int>> pp(list.size(), make_pair(-1, INT_MAX)); // fringe 노드의 predecessor를 저장하는 배열 path를 backtrack하기위한용도
	vector<int> dist(list.size(), INT_MAX); // A로부터의 거리를 저장하는 배열
	vector<int> fringe(list.size()); //fringe set에 있는 노드인지 검사하기위한 배열
	dist[arr[A.cityNum]] = 0; // A에서 A까지 의 거리는 0으로 정의
	visit[arr[A.cityNum]] = 1; //tree에 시작 vertex만 있음
	int treeSize = 1; // 시작 vertex 1개
	int y = arr[A.cityNum]; // y는 predecssor를 임시로 저장할 변수 지금은 시작 vertex.
	for (int i = 0; i < A.neighbor.size(); i++) { // 시작점의 인접vertex를 fringe에 넣는다
		Minpq.push(make_pair(A.neighbor[i].second, A.neighbor[i].first)); //(dist, city Number)
		pp[arr[A.neighbor[i].first]].first = arr[A.cityNum]; // pp [i]= (i의 predecessor ,A로부터의거리)로 정의
		pp[arr[A.neighbor[i].first]].second = A.neighbor[i].second;
		fringe[arr[A.neighbor[i].first]] = 1; //fringe set에 들어있다고 check
	}
	while (!Minpq.empty()) {
		int u = arr[Minpq.top().second];
		int w = Minpq.top().first;
		y = pp[arr[Minpq.top().second]].first;
		if (visit[u]) { Minpq.pop(); continue; } // 이미 최적의 값이 tree에 있으므로 무시한다.
		Minpq.pop();
		treeSize++; // fringe에서 pop되는것은 tree set에 추가되는것이므로 tree의 사이즈를 업데이트
		visit[u] = 1;
		if (dist[u] > w) { dist[u] = w; } //더 짧은 거리가 있다면 그 거리로 업데이트한다.
		for (int i = 0; i < list[u].neighbor.size(); i++) {
			if (!visit[arr[list[u].neighbor[i].first]]) { //tree에 있는 vertex는 fringe에 넣지않는다.
														  /*decrease key*/
				if (fringe[arr[list[u].neighbor[i].first]]) { // 이미 fringe에 있다면 predecessor를 업데이트할지를 본다  즉 decreaseKey. 
					if (pp[arr[list[u].neighbor[i].first]].second > list[u].neighbor[i].second + dist[u]) { //source로부터의 거리가 더 작다면 기존의 
						Minpq.push(make_pair(list[u].neighbor[i].second + dist[u], list[u].neighbor[i].first));//predecssor보다 현재 predecessor가 최적이므로 변경
						pp[arr[list[u].neighbor[i].first]].first = u; // 해당 vertex의 predecessor는 u이다
						pp[arr[list[u].neighbor[i].first]].second = list[u].neighbor[i].second + dist[u];	// 해당 vertex로부터 시작점까지의 거리= 시작점에서 predecessor까지 거리 + predecessor랑 해당 vertex까지 거리 
					}
					//변경할필요가없으면 아무작업하지않아도 됨
				}
				else { //fringe에 없는 vertex라면 추가
					Minpq.push(make_pair(list[u].neighbor[i].second + dist[u], list[u].neighbor[i].first));
					pp[arr[list[u].neighbor[i].first]].first = u;
					pp[arr[list[u].neighbor[i].first]].second = list[u].neighbor[i].second + dist[u];
					fringe[arr[list[u].neighbor[i].first]] = 1; //fringe set에 있음을 체크
				}
			}
		}
		if (u == arr[B.cityNum]) break; //도착점이 tree set에 들어갔다면 루프를 종료
	}
	if (treeSize == 1 || pp[arr[B.cityNum]].first == -1) return -1; // treeSize가 1인경우는 source가 고립. 도착점의 부모가 setting되지 않은것은 경로가 없는것
	int t = arr[B.cityNum]; // 도착점부터 백트랙킹
	stack<int> s;
	while (t != -1) { // 저장된 predecessor를 찾아 올라가면서 path에 저장
		path.push_back(t);
		t = pp[t].first;
	}

	lastDist = dist[arr[B.cityNum]]; //dist에 저장된 값이 최소거리이다. 10^6을넘어가면 못가는것으로 간주한다.
	return treeSize; // tree의 사이즈를 리턴
}

//main
int main() {
	/* I/O 속도 개선 . similar to printf scanf */
	ios::sync_with_stdio(false); //c의 표준입출력과의 출력동기화를 false
	cin.tie(0); //I/O stream을 untie // cin.tie(&cout)이 default임
	cout.tie(0);

	int area_num; //지역번호
	string area_name; //지역명
	int flood; //침수여부 1: 침수 0: 정상

	int n, m, q; // n: vertex수 m: 간선 수 q: 질의 수
	cin >> n >> m >> q; //input n<=10^5 m<=2*10^5 q<=50


	list.resize(n);

	int dist, area_1, area_2; //dist <= 5000 

	for (int i = 0; i < n; i++) { //입력받은순서로 적당히 vertex를 0부터 n-1까지 numbering
		cin >> area_num >> area_name >> flood;
		list[i].cityName = area_name; // 지역이름
		list[i].cityNum = area_num; //지역 번호
		list[i].isFlood = flood; //침수 여부
		arr[area_num] = i; // area_num의 정보는  list의 i 번째 index에 있음 즉 i번노드
	}
	for (int i = 0; i < m; i++) {
		cin >> area_1 >> area_2 >> dist;
		if (list[arr[area_1]].isFlood == 0 && list[arr[area_2]].isFlood == 0) { //침수지역이있다면 그 지역으로 가는 길은 없다고 치자.
			list[arr[area_1]].neighbor.push_back(make_pair(area_2, dist)); //area_1의 neighbor area_2이며 두 도시의 거리는 dist이다
			list[arr[area_2]].neighbor.push_back(make_pair(area_1, dist)); // 같은정보를 area_2의 vertex에도 저장
		}
	}
	char c;
	for (int i = 0; i < q; i++) {
		int ans, num_tree; vector<int> path;
		cin >> c >> area_1 >> area_2;
		num_tree = dijkstra(list[arr[area_1]], list[arr[area_2]], path, ans);
		if (c == 'A') { //질의 A : 주어진 도시사이의 최소거리와 목적직 까지의 Tree vertex 상의 지역수, 출발,도착 지역명을 출력
			if (ans>over || num_tree == -1) cout << "None\n";
			else {
				cout << num_tree << " " << ans << " " << list[arr[area_1]].cityName << " "
					<< list[arr[area_2]].cityName << "\n";
			}
		}
		else if (c == 'B') { //질의 B : path상의 지역수와 각 지역들의 지역번호를 순차적(path순)으로 출력
			if (ans>over || num_tree == -1) cout << "None\n";
			else {
				cout << num_tree << " ";
				for (int i = path.size() - 1; i >= 0; i--) // 도착점부터 저장했으므로 거꾸로 출력한다.
					cout << list[path[i]].cityNum << " ";
				cout << "\n";
			}
		}
	}

}