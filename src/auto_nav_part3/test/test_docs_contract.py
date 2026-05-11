from pathlib import Path


def test_topic_contract_document_exists():
    repo_root = Path(__file__).resolve().parents[3]
    contract = repo_root / 'docs' / 'TOPICS.md'
    assert contract.exists()
    text = contract.read_text()
    assert '/part3/mapping/start' in text
    assert '/part3/waypoint/start' in text
    assert '/part3/system/state' in text


def test_task_allocation_has_three_members():
    repo_root = Path(__file__).resolve().parents[3]
    allocation = repo_root / 'docs' / 'TASK_ALLOCATION.md'
    text = allocation.read_text()
    assert 'Member 1' in text
    assert 'Member 2' in text
    assert 'Member 3' in text
