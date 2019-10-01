/*
Dijkstra Algorithm�� �̿��� �׺���̼� ���α׷� ���� ����

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
struct city { //vertex�� ������ ������ ����ü
	int cityNum; //vertex�� ������ȣ 100,000 ~ 999,999
	string cityName; //vertex�� ������
	vector< pair<int, int> > neighbor; //(neighbor, dist) ������ vertex�� ������ȣ �� edge weight�� ����
	int isFlood; // ħ������
};
#define over 1000000  //�Ÿ��� 10^6 �ʰ��ҽ� ��ΰ��������� ����
vector<int> visit; // vertex�� tree�� ������ �˻��ϴ� �迭

				   /*arr[1000000]���� ��������Ʈ�� ����� ū���̴� �������� vertex����ŭ�� ����Ʈ�� ����°ͺ���
				   ��ȿ�����̹Ƿ� ������ȣ�� ���� 1:1 �ؽ̹迭�� ������� �ߺ��� ������ȣ���������� collision�� ����������� */
vector<int> arr(1000000); //list�� 1:1 correspondence, hashing
vector<city> list; //vertex���� ��������Ʈ						  

				   /*Dijkstra Algorithm*/
int dijkstra(city A, city B, vector<int> &path, int &lastDist) { // �Ű������� ���,���� ����, vertex�� ��������
	priority_queue< pair<int, int>, vector< pair <int, int> >, greater< pair<int, int> > > Minpq; // fringe vertex�� ������ min heap(dist,cit number)
	visit.assign(list.size(), 0); // tree�� vertex�� ����
	vector<pair<int, int>> pp(list.size(), make_pair(-1, INT_MAX)); // fringe ����� predecessor�� �����ϴ� �迭 path�� backtrack�ϱ����ѿ뵵
	vector<int> dist(list.size(), INT_MAX); // A�κ����� �Ÿ��� �����ϴ� �迭
	vector<int> fringe(list.size()); //fringe set�� �ִ� ������� �˻��ϱ����� �迭
	dist[arr[A.cityNum]] = 0; // A���� A���� �� �Ÿ��� 0���� ����
	visit[arr[A.cityNum]] = 1; //tree�� ���� vertex�� ����
	int treeSize = 1; // ���� vertex 1��
	int y = arr[A.cityNum]; // y�� predecssor�� �ӽ÷� ������ ���� ������ ���� vertex.
	for (int i = 0; i < A.neighbor.size(); i++) { // �������� ����vertex�� fringe�� �ִ´�
		Minpq.push(make_pair(A.neighbor[i].second, A.neighbor[i].first)); //(dist, city Number)
		pp[arr[A.neighbor[i].first]].first = arr[A.cityNum]; // pp [i]= (i�� predecessor ,A�κ����ǰŸ�)�� ����
		pp[arr[A.neighbor[i].first]].second = A.neighbor[i].second;
		fringe[arr[A.neighbor[i].first]] = 1; //fringe set�� ����ִٰ� check
	}
	while (!Minpq.empty()) {
		int u = arr[Minpq.top().second];
		int w = Minpq.top().first;
		y = pp[arr[Minpq.top().second]].first;
		if (visit[u]) { Minpq.pop(); continue; } // �̹� ������ ���� tree�� �����Ƿ� �����Ѵ�.
		Minpq.pop();
		treeSize++; // fringe���� pop�Ǵ°��� tree set�� �߰��Ǵ°��̹Ƿ� tree�� ����� ������Ʈ
		visit[u] = 1;
		if (dist[u] > w) { dist[u] = w; } //�� ª�� �Ÿ��� �ִٸ� �� �Ÿ��� ������Ʈ�Ѵ�.
		for (int i = 0; i < list[u].neighbor.size(); i++) {
			if (!visit[arr[list[u].neighbor[i].first]]) { //tree�� �ִ� vertex�� fringe�� �����ʴ´�.
														  /*decrease key*/
				if (fringe[arr[list[u].neighbor[i].first]]) { // �̹� fringe�� �ִٸ� predecessor�� ������Ʈ������ ����  �� decreaseKey. 
					if (pp[arr[list[u].neighbor[i].first]].second > list[u].neighbor[i].second + dist[u]) { //source�κ����� �Ÿ��� �� �۴ٸ� ������ 
						Minpq.push(make_pair(list[u].neighbor[i].second + dist[u], list[u].neighbor[i].first));//predecssor���� ���� predecessor�� �����̹Ƿ� ����
						pp[arr[list[u].neighbor[i].first]].first = u; // �ش� vertex�� predecessor�� u�̴�
						pp[arr[list[u].neighbor[i].first]].second = list[u].neighbor[i].second + dist[u];	// �ش� vertex�κ��� ������������ �Ÿ�= ���������� predecessor���� �Ÿ� + predecessor�� �ش� vertex���� �Ÿ� 
					}
					//�������ʿ䰡������ �ƹ��۾������ʾƵ� ��
				}
				else { //fringe�� ���� vertex��� �߰�
					Minpq.push(make_pair(list[u].neighbor[i].second + dist[u], list[u].neighbor[i].first));
					pp[arr[list[u].neighbor[i].first]].first = u;
					pp[arr[list[u].neighbor[i].first]].second = list[u].neighbor[i].second + dist[u];
					fringe[arr[list[u].neighbor[i].first]] = 1; //fringe set�� ������ üũ
				}
			}
		}
		if (u == arr[B.cityNum]) break; //�������� tree set�� ���ٸ� ������ ����
	}
	if (treeSize == 1 || pp[arr[B.cityNum]].first == -1) return -1; // treeSize�� 1�ΰ��� source�� ��. �������� �θ� setting���� �������� ��ΰ� ���°�
	int t = arr[B.cityNum]; // ���������� ��Ʈ��ŷ
	stack<int> s;
	while (t != -1) { // ����� predecessor�� ã�� �ö󰡸鼭 path�� ����
		path.push_back(t);
		t = pp[t].first;
	}

	lastDist = dist[arr[B.cityNum]]; //dist�� ����� ���� �ּҰŸ��̴�. 10^6���Ѿ�� �����°����� �����Ѵ�.
	return treeSize; // tree�� ����� ����
}

//main
int main() {
	/* I/O �ӵ� ���� . similar to printf scanf */
	ios::sync_with_stdio(false); //c�� ǥ������°��� ��µ���ȭ�� false
	cin.tie(0); //I/O stream�� untie // cin.tie(&cout)�� default��
	cout.tie(0);

	int area_num; //������ȣ
	string area_name; //������
	int flood; //ħ������ 1: ħ�� 0: ����

	int n, m, q; // n: vertex�� m: ���� �� q: ���� ��
	cin >> n >> m >> q; //input n<=10^5 m<=2*10^5 q<=50


	list.resize(n);

	int dist, area_1, area_2; //dist <= 5000 

	for (int i = 0; i < n; i++) { //�Է¹��������� ������ vertex�� 0���� n-1���� numbering
		cin >> area_num >> area_name >> flood;
		list[i].cityName = area_name; // �����̸�
		list[i].cityNum = area_num; //���� ��ȣ
		list[i].isFlood = flood; //ħ�� ����
		arr[area_num] = i; // area_num�� ������  list�� i ��° index�� ���� �� i�����
	}
	for (int i = 0; i < m; i++) {
		cin >> area_1 >> area_2 >> dist;
		if (list[arr[area_1]].isFlood == 0 && list[arr[area_2]].isFlood == 0) { //ħ���������ִٸ� �� �������� ���� ���� ���ٰ� ġ��.
			list[arr[area_1]].neighbor.push_back(make_pair(area_2, dist)); //area_1�� neighbor area_2�̸� �� ������ �Ÿ��� dist�̴�
			list[arr[area_2]].neighbor.push_back(make_pair(area_1, dist)); // ���������� area_2�� vertex���� ����
		}
	}
	char c;
	for (int i = 0; i < q; i++) {
		int ans, num_tree; vector<int> path;
		cin >> c >> area_1 >> area_2;
		num_tree = dijkstra(list[arr[area_1]], list[arr[area_2]], path, ans);
		if (c == 'A') { //���� A : �־��� ���û����� �ּҰŸ��� ������ ������ Tree vertex ���� ������, ���,���� �������� ���
			if (ans>over || num_tree == -1) cout << "None\n";
			else {
				cout << num_tree << " " << ans << " " << list[arr[area_1]].cityName << " "
					<< list[arr[area_2]].cityName << "\n";
			}
		}
		else if (c == 'B') { //���� B : path���� �������� �� �������� ������ȣ�� ������(path��)���� ���
			if (ans>over || num_tree == -1) cout << "None\n";
			else {
				cout << num_tree << " ";
				for (int i = path.size() - 1; i >= 0; i--) // ���������� ���������Ƿ� �Ųٷ� ����Ѵ�.
					cout << list[path[i]].cityNum << " ";
				cout << "\n";
			}
		}
	}

}